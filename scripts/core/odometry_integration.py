"""
Rover Odometry Integration Module

Provides a simple Python API to the fused odometry system.
Can be used independently from ROS2 or alongside the ROS2 publisher node.

Usage:
    from odometry_integration import get_odometry, update_odometry

    # Update odometry (called from main loop ~ 50 Hz)
    state = update_odometry(
        rpm_left=sensor_packet.rpm_left,
        rpm_right=sensor_packet.rpm_right,
        gyro_z_deg_s=sensor_packet.gyro_z,
        mag_heading_deg=calculate_compass_heading(...),
        steering_angle_deg=current_steer_cmd,
    )
    
    # Query current state
    x, y = get_position()
    heading = get_heading_deg()
    v_lin = get_linear_velocity()
    v_ang = get_angular_velocity()

Author: Rover Control System (March 2026)
"""

import time
import math
from typing import Optional, Tuple

from fused_odometry import FusedOdometry, create_odometry_measurement

# Global odometry instance
_odometry_engine: Optional[FusedOdometry] = None
_odometry_lock = None
_init_complete = False


def init_odometry() -> None:
    """Initialize fused odometry system. Call once at startup."""
    global _odometry_engine, _odometry_lock, _init_complete
    
    import threading
    
    if _odometry_engine is not None:
        return
    
    _odometry_engine = FusedOdometry()
    _odometry_lock = threading.RLock()
    _init_complete = True
    print("✅ [Odometry] Fused odometry system initialized")


def update_odometry(
    rpm_left: float,
    rpm_right: float,
    gyro_z_deg_s: float,
    mag_heading_deg: Optional[float] = None,
    steering_angle_deg: float = 0.0,
    accel_x: float = 0.0,
) -> Optional['RoverOdometryState']:
    """
    Update odometry estimate with latest sensor measurements.
    
    Args:
        rpm_left: Left rear motor RPM
        rpm_right: Right rear motor RPM
        gyro_z_deg_s: Gyro Z-axis angular velocity (deg/s)
        mag_heading_deg: Magnetometer-derived heading (deg), or None if not available
        steering_angle_deg: Front steering angle (deg)
        accel_x: Forward acceleration (g)
    
    Returns:
        Updated RoverOdometryState with position and velocity estimates
    """
    global _odometry_engine
    
    if _odometry_engine is None:
        return None
    
    with _odometry_lock:
        measurement = create_odometry_measurement(
            timestamp_s=time.time(),
            rpm_left=rpm_left,
            rpm_right=rpm_right,
            gyro_z_deg_s=gyro_z_deg_s,
            mag_heading_deg=mag_heading_deg,
            steering_angle_deg=steering_angle_deg,
            accel_x=accel_x,
        )
        
        return _odometry_engine.update(measurement)


def get_position() -> Tuple[float, float]:
    """Get current (x, y) position estimate in meters."""
    global _odometry_engine
    if _odometry_engine is None:
        return (0.0, 0.0)
    
    with _odometry_lock:
        return _odometry_engine.get_position()


def get_heading() -> float:
    """Get current heading estimate in radians [-π, π]."""
    global _odometry_engine
    if _odometry_engine is None:
        return 0.0
    
    with _odometry_lock:
        return _odometry_engine.get_heading()


def get_heading_deg() -> float:
    """Get current heading estimate in degrees."""
    global _odometry_engine
    if _odometry_engine is None:
        return 0.0
    
    with _odometry_lock:
        return _odometry_engine.get_heading_deg()


def get_linear_velocity() -> float:
    """Get current linear velocity estimate (m/s)."""
    global _odometry_engine
    if _odometry_engine is None:
        return 0.0
    
    with _odometry_lock:
        return _odometry_engine.get_linear_velocity()


def get_angular_velocity() -> float:
    """Get current angular velocity estimate (rad/s)."""
    global _odometry_engine
    if _odometry_engine is None:
        return 0.0
    
    with _odometry_lock:
        return _odometry_engine.get_angular_velocity()


def get_gyro_bias() -> float:
    """Get estimated gyro Z-axis bias (rad/s)."""
    global _odometry_engine
    if _odometry_engine is None:
        return 0.0
    
    with _odometry_lock:
        return _odometry_engine.get_gyro_bias()


def get_state():
    """Get complete odometry state (RoverOdometryState object)."""
    global _odometry_engine
    if _odometry_engine is None:
        return None
    
    with _odometry_lock:
        return _odometry_engine.get_state()


def get_covariance_diag():
    """Get diagonal of state covariance matrix (position, heading, velocity uncertainties)."""
    global _odometry_engine
    if _odometry_engine is None:
        return {}
    
    with _odometry_lock:
        P = _odometry_engine.get_covariance()
        return {
            'pos_x_std_m': math.sqrt(max(0.0, P[0, 0])),
            'pos_y_std_m': math.sqrt(max(0.0, P[1, 1])),
            'heading_std_rad': math.sqrt(max(0.0, P[2, 2])),
            'heading_std_deg': math.degrees(math.sqrt(max(0.0, P[2, 2]))),
            'v_linear_std': math.sqrt(max(0.0, P[3, 3])),
            'v_angular_std': math.sqrt(max(0.0, P[4, 4])),
        }


def reset_pose(x: float = 0.0, y: float = 0.0, theta_deg: float = 0.0) -> None:
    """
    Reset pose estimate to a known position.
    
    Args:
        x: X position (meters)
        y: Y position (meters)
        theta_deg: Heading (degrees), will be converted to radians
    """
    global _odometry_engine
    if _odometry_engine is None:
        return
    
    with _odometry_lock:
        theta_rad = math.radians(theta_deg)
        _odometry_engine.reset(x, y, theta_rad)
        print(f"✅ [Odometry] Reset pose to ({x:.3f}, {y:.3f}), heading={theta_deg:.1f}°")


def increase_uncertainty(pos_std_m: float = 0.1, heading_std_deg: float = 10.0) -> None:
    """
    Increase state uncertainty (e.g., after wheel slip or environment change).
    Useful when the rover detects likely odometry drift.
    
    Args:
        pos_std_m: Additional position uncertainty (meters)
        heading_std_deg: Additional heading uncertainty (degrees)
    """
    global _odometry_engine
    if _odometry_engine is None:
        return
    
    with _odometry_lock:
        _odometry_engine.set_uncertainty(pos_std_m, heading_std_deg)


def get_diagnostics_dict() -> dict:
    """
    Get complete odometry diagnostics as a dictionary.
    Useful for logging, telemetry, or UI display.
    
    Returns:
        Dictionary with all pose, velocity, and uncertainty information
    """
    global _odometry_engine
    if _odometry_engine is None:
        return {}
    
    with _odometry_lock:
        state = _odometry_engine.get_state()
        covariances = get_covariance_diag()
        
        return {
            # Position
            'x_m': state.x,
            'y_m': state.y,
            'heading_deg': math.degrees(state.theta),
            'heading_rad': state.theta,
            
            # Velocity
            'v_linear_m_s': state.v_linear,
            'v_angular_rad_s': state.v_angular,
            
            # Bias
            'gyro_bias_rad_s': _odometry_engine.get_gyro_bias(),
            
            # Uncertainty
            'std_pos_x_m': covariances['pos_x_std_m'],
            'std_pos_y_m': covariances['pos_y_std_m'],
            'std_heading_deg': covariances['heading_std_deg'],
            'std_v_linear': covariances['v_linear_std'],
            'std_v_angular': covariances['v_angular_std'],
            
            # Timestamp
            'timestamp_s': state.timestamp_s,
        }


# Convenience: Also expose the state object class for type hints
from fused_odometry import RoverOdometryState

__all__ = [
    'init_odometry',
    'update_odometry',
    'get_position',
    'get_heading',
    'get_heading_deg',
    'get_linear_velocity',
    'get_angular_velocity',
    'get_gyro_bias',
    'get_state',
    'get_covariance_diag',
    'reset_pose',
    'increase_uncertainty',
    'get_diagnostics_dict',
    'RoverOdometryState',
]
