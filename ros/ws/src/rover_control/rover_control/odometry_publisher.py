"""
ROS2 Odometry Publisher Node for Fused Odometry

Subscribes to:
  - None directly; reads from rover_odometry global state

Publishes:
  - /odom (nav_msgs/Odometry) @ 10 Hz
  - /rover_odometry_state (custom RoverState) @ 50 Hz
  - /tf (geometry_msgs/TransformStamped) odom→base_footprint

Integrates fused_odometry.py sensor fusion with ROS2.

Author: Rover Control System (March 2026)
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist, Vector3
from tf2_ros import TransformBroadcaster
import tf_transformations

# Import fused odometry (parent directory)
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.fused_odometry import FusedOdometry, SensorMeasurements
from core.pico_sensor_reader import get_sensor_packet
from core.compass_calibration import load_calibration, compute_heading_degrees


class OdometryPublisher(Node):
    """ROS2 node for fused odometry estimation and publishing."""
    
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Get parameters
        self.declare_parameter('publish_rate_hz', 10)  # Odometry nav_msgs publication rate
        self.declare_parameter('sensor_read_rate_hz', 50)  # Sensor reading rate
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('use_compass', True)
        self.declare_parameter('compass_cal_file', 'scripts/diagnostics/compass_cal.json')
        
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.sensor_read_rate_hz = self.get_parameter('sensor_read_rate_hz').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.use_compass = self.get_parameter('use_compass').value
        self.compass_cal_file = self.get_parameter('compass_cal_file').value
        
        # QoS profile (reliable, keep last 10 messages)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self._odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self._tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize sensor fusion
        self._odometry = FusedOdometry()
        
        # Load compass calibration if used
        self._compass_cal = None
        if self.use_compass:
            try:
                self._compass_cal = load_calibration(self.compass_cal_file)
                self.get_logger().info(f"Loaded compass calibration from {self.compass_cal_file}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load compass calibration: {e}")
                self._compass_cal = None
        
        # Timing
        self._last_odom_pub_time = time.time()
        self._last_sensor_read_time = time.time()
        self._odom_pub_interval = 1.0 / max(1, self.publish_rate_hz)
        self._sensor_read_interval = 1.0 / max(1, self.sensor_read_rate_hz)
        
        # Sequence number for header
        self._seq = 0
        
        # State monitoring
        self._measurement_count = 0
        self._last_diagnostic_time = time.time()
        
        # Create main timer (runs at sensor rate)
        timer_period = 1.0 / max(1, self.sensor_read_rate_hz)
        self._timer = self.create_timer(timer_period, self._timer_callback)
        
        self.get_logger().info(f"Odometry publisher initialized (publish @ {self.publish_rate_hz} Hz, "
                              f"sensor read @ {self.sensor_read_rate_hz} Hz)")
    
    def _timer_callback(self) -> None:
        """
        Main timer callback: read sensors and update odometry.
        Publishes Odometry msg at reduced rate (publish_rate_hz).
        """
        try:
            current_time = time.time()
            
            # Read latest sensor packet from Pico
            sensor_packet = get_sensor_packet()
            if not sensor_packet:
                self.get_logger().warn_once("No sensor packet available yet")
                return
            
            # Convert sensor packet to odometry measurement
            measurement = self._sensor_packet_to_measurement(sensor_packet)
            
            # Update odometry filter
            state = self._odometry.update(measurement)
            self._measurement_count += 1
            
            # Publish odometry at reduced rate
            if current_time - self._last_odom_pub_time >= self._odom_pub_interval:
                self._publish_odometry(state, current_time)
                self._publish_tf(state, current_time)
                self._last_odom_pub_time = current_time
            
            # Periodic diagnostics
            if current_time - self._last_diagnostic_time > 10.0:
                self._log_diagnostics(state)
                self._last_diagnostic_time = current_time
        
        except Exception as e:
            self.get_logger().error(f"Error in odometry update: {e}", throttle_duration_sec=5.0)
    
    def _sensor_packet_to_measurement(self, sensor_packet) -> SensorMeasurements:
        """
        Convert Pico sensor packet to FusedOdometry measurement.
        
        sensor_packet has attributes:
          - rpm_left, rpm_right (float)
          - gyro_x, gyro_y, gyro_z (float, deg/s)
          - mag_x, mag_y, mag_z (float, Gauss)
          - accel_x, accel_y, accel_z (float, g)
          - timestamp_ms (int)
        """
        timestamp_s = sensor_packet.timestamp_ms / 1000.0
        
        # Encoder RPM
        rpm_left = sensor_packet.rpm_left
        rpm_right = sensor_packet.rpm_right
        
        # Gyro Z (yaw rate)
        gyro_z_deg_s = sensor_packet.gyro_z
        
        # Magnetometer heading (if available and calibrated)
        mag_heading_deg = None
        mag_quality = 0.0
        if self.use_compass and self._compass_cal:
            try:
                mag_heading_deg, quality = compute_heading_degrees(
                    sensor_packet.mag_x,
                    sensor_packet.mag_y,
                    sensor_packet.mag_z,
                    self._compass_cal
                )
                mag_quality = min(1.0, quality) if hasattr(quality, '__float__') else 1.0
            except Exception as e:
                self.get_logger().debug(f"Compass computation error: {e}")
                mag_heading_deg = None
        
        # Steering angle (TODO: get from follow_line.py or store globally)
        # For now, assume 0 (straight)
        steering_angle_deg = 0.0
        
        # Acceleration
        accel_x = sensor_packet.accel_x
        
        return SensorMeasurements(
            timestamp_s=timestamp_s,
            rpm_left=rpm_left,
            rpm_right=rpm_right,
            gyro_z_deg_s=gyro_z_deg_s,
            mag_heading_deg=mag_heading_deg,
            mag_quality=mag_quality,
            steering_angle_deg=steering_angle_deg,
            accel_x=accel_x,
        )
    
    def _publish_odometry(self, state, current_time: float) -> None:
        """Publish nav_msgs/Odometry message."""
        odom = Odometry()
        
        # Header
        odom.header.seq = self._seq
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw angle)
        quat = tf_transformations.quaternion_from_euler(0, 0, state.theta)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Pose covariance (from uncertainty estimate)
        P = self._odometry.get_covariance()
        odom.pose.covariance = self._create_pose_covariance(P)
        
        # Twist (velocity)
        odom.twist.twist.linear = Vector3(x=state.v_linear, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=state.v_angular)
        
        # Twist covariance
        odom.twist.covariance = self._create_twist_covariance(P)
        
        self._odom_pub.publish(odom)
        self._seq += 1
    
    def _publish_tf(self, state, current_time: float) -> None:
        """Publish TF transform from odom to base_footprint."""
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Position
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = 0.0
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, state.theta)
        t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        self._tf_broadcaster.sendTransform(t)
    
    def _create_pose_covariance(self, P: 'np.ndarray') -> list:
        """Create 6×6 pose covariance from state covariance matrix."""
        # ROS Odometry pose covariance: [x, y, z, roll, pitch, yaw]
        # Our state: [x, y, theta, v_lin, v_ang, gyro_bias]
        cov = [0.0] * 36
        cov[0] = P[0, 0]   # x
        cov[7] = P[1, 1]   # y
        cov[14] = 0.01     # z (always 0, use small constant)
        cov[21] = 0.01     # roll
        cov[28] = 0.01     # pitch
        cov[35] = P[2, 2]  # yaw
        
        # Cross-covariances
        cov[1] = P[0, 1]   # x-y
        cov[6] = P[0, 1]   # y-x
        
        return cov
    
    def _create_twist_covariance(self, P: 'np.ndarray') -> list:
        """Create 6×6 twist covariance from state covariance matrix."""
        # Twist covariance: [v_x, v_y, v_z, w_x, w_y, w_z]
        cov = [0.0] * 36
        cov[0] = P[3, 3]   # v_x (linear)
        cov[7] = 0.01      # v_y (lateral, assumed small)
        cov[14] = 0.01     # v_z (vertical)
        cov[28] = P[4, 4]  # w_z (yaw rate)
        cov[21] = 0.01     # w_x
        cov[35] = 0.01     # w_y
        
        return cov
    
    def _log_diagnostics(self, state) -> None:
        """Log diagnostics every 10 seconds."""
        P = self._odometry.get_covariance()
        
        self.get_logger().info(
            f"Odometry: pos=({state.x:.3f}, {state.y:.3f}), "
            f"heading={state.theta:.3f} rad ({self._odometry.get_heading_deg():.1f}°), "
            f"v_lin={state.v_linear:.3f} m/s, v_ang={state.v_angular:.3f} rad/s, "
            f"gyro_bias={self._odometry.get_gyro_bias():.4f} rad/s, "
            f"meas_count={self._measurement_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
