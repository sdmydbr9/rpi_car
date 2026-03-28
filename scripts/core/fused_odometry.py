"""
Fused Odometry Module - UKF-based sensor fusion for rover pose estimation

Combines:
- Rear encoders (linear velocity via RPM)
- Gyro Z-axis (angular velocity)  
- Magnetometer/Compass (absolute heading)
- Steering angle (motion model constraint)
- Front accelerometer (velocity validation)

Implements Unscented Kalman Filter (UKF) for robust pose estimation.

Author: Rover Control System (March 2026)
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


@dataclass
class RoverOdometryState:
    """Current rover position and orientation."""
    x: float = 0.0              # meters (East)
    y: float = 0.0              # meters (North)
    theta: float = 0.0          # radians (yaw, 0=East, π/2=North)
    v_linear: float = 0.0       # m/s forward velocity
    v_angular: float = 0.0      # rad/s yaw rate
    timestamp_s: float = field(default_factory=time.time)
    
    def pose_tuple(self) -> Tuple[float, float, float]:
        """Return (x, y, theta) position tuple."""
        return (self.x, self.y, self.theta)
    
    def velocity_tuple(self) -> Tuple[float, float]:
        """Return (v_linear, v_angular) velocity tuple."""
        return (self.v_linear, self.v_angular)


@dataclass
class SensorMeasurements:
    """Single-sample sensor measurement from rover."""
    timestamp_s: float
    rpm_left: float                 # rear-left motor RPM
    rpm_right: float                # rear-right motor RPM
    gyro_z_deg_s: float             # yaw rate (deg/s)
    mag_heading_deg: Optional[float]  # magnetometer heading (deg), None if unavailable
    mag_quality: float = 1.0        # confidence 0-1 (lower if noisy)
    steering_angle_deg: float = 0.0 # front steering servo angle
    accel_x: float = 0.0            # forward acceleration (g)
    accel_y: float = 0.0            # lateral acceleration (g)


class FusedOdometry:
    """
    Unscented Kalman Filter for rover pose estimation.
    
    State vector: [x, y, theta, v_linear, v_angular, gyro_bias_z]
    6-D state: position (2) + heading (1) + velocity (1) + angularvel (1) + bias (1)
    
    Motion model: Ackermann kinematic model
    - Linear velocity from encoder RPM
    - Angular velocity from steering angle + velocity (kinematic constraints)
    - Gyro provides direct measurement + bias tracking
    
    Measurement model:
    - Magnetometer: heading (theta)
    - Gyro: angular velocity (v_angular, biased)
    """
    
    # Rover geometry (from URDF / measurements)
    WHEELBASE_M = 0.210       # Distance from rear axle to front steering axle
    WHEEL_RADIUS_M = 0.0375   # Wheel radius (75mm diameter)
    TRACK_WIDTH_M = 0.172     # Distance between rear wheels
    
    # UKF parameters
    ALPHA = 1e-3              # Spread of sigma points (1e-4 to 1)
    BETA = 2.0                # Optimal for Gaussian: 2
    KAPPA = 0.0               # Secondary scaling (0 for 6-D state)
    
    # Motion model noise (process covariance)
    SIG_V_LINEAR = 0.15       # m/s (encoder + slippage uncertainty)
    SIG_V_ANGULAR = 0.10      # rad/s (steering + kinematic coupling)
    SIG_GYRO_BIAS = 0.001     # rad/s (gyro bias drift)
    SIG_ACCEL_X = 0.3         # m/s^2 (unused currently, for future accel fusion)
    
    # Measurement noise (measurement covariance)
    SIG_HEADING_MAG = math.radians(5.0)   # 5° magnetometer heading std
    SIG_GYRO_Z = math.radians(1.0)        # 1° gyro rate std (well-calibrated)
    SIG_ACCEL = 0.05                      # (unused currently)
    
    def __init__(self):
        """Initialize filter state and covariance."""
        # State: [x, y, theta, v_linear, v_angular, gyro_bias_z]
        self._state = np.zeros(6)  # x, y, theta, v_lin, v_ang, gyro_bias
        self._state[2] = 0.0  # theta = 0 (East-facing)
        
        # Covariance matrix (uncertainty in state)
        self._P = np.eye(6) * 0.01  # Start with modest uncertainty
        self._P[2, 2] = math.radians(10)**2  # Higher heading uncertainty
        
        # Last measurement time for dt calculation
        self._last_time_s = time.time()
        
        # UKF constants
        n = 6  # state dimension
        self._lambda = self.ALPHA**2 * (n + self.KAPPA) - n
        self._gamma = math.sqrt(n + self._lambda)
        
        # Weights for sigma points
        self._Wm = np.zeros(2*n + 1)  # Weights for mean
        self._Wc = np.zeros(2*n + 1)  # Weights for covariance
        
        self._Wm[0] = self._lambda / (n + self._lambda)
        self._Wc[0] = self._lambda / (n + self._lambda) + (1 - self.ALPHA**2 + self.BETA)
        for i in range(1, 2*n + 1):
            self._Wm[i] = 1.0 / (2 * (n + self._lambda))
            self._Wc[i] = 1.0 / (2 * (n + self._lambda))
        
        # Initialize measurements log for debugging
        self._meas_log = []
    
    def update(self, measurement: SensorMeasurements) -> RoverOdometryState:
        """
        Process one sensor measurement and update pose estimate.
        
        Args:
            measurement: Single measurement from all sensors
            
        Returns:
            Updated rover pose and velocity
        """
        dt = measurement.timestamp_s - self._last_time_s
        if dt <= 0:
            dt = 0.01  # Fallback if clock goes backward
        if dt > 1.0:
            dt = 0.05  # Fallback if time jump (e.g., sensor timeout)
        
        self._last_time_s = measurement.timestamp_s
        
        # --- PREDICTION STEP (Motion Model) ---
        self._predict(measurement, dt)
        
        # --- MEASUREMENT UPDATE STEP ---
        if measurement.mag_heading_deg is not None and measurement.mag_quality > 0.5:
            self._update_heading(measurement)
        
        # Gyro Z provides both direct measurement AND helps with bias tracking
        self._update_gyro_rate(measurement)
        
        # Log for diagnostics (cap at 200 entries to prevent memory leak)
        self._meas_log.append({
            't': measurement.timestamp_s,
            'rpm_l': measurement.rpm_left,
            'rpm_r': measurement.rpm_right,
            'gyro_z': measurement.gyro_z_deg_s,
            'mag_h': measurement.mag_heading_deg,
            'steer': measurement.steering_angle_deg,
        })
        if len(self._meas_log) > 200:
            self._meas_log = self._meas_log[-200:]
        
        return self.get_state()
    
    def _predict(self, meas: SensorMeasurements, dt: float) -> None:
        """
        Prediction step: advance state based on motion model.
        Uses UKF sigma point propagation.
        """
        # Generate sigma points from current state
        sigma_points = self._generate_sigma_points()
        
        # Propagate sigma points through motion model
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(len(sigma_points)):
            sigma_points_pred[i] = self._motion_model(sigma_points[i], meas, dt)
        
        # Recover mean and covariance from propagated sigma points
        self._state = np.average(sigma_points_pred, axis=0, weights=self._Wm)
        
        # Wrap heading to [-π, π]
        self._state[2] = self._wrap_angle(self._state[2])
        
        # Compute predicted covariance from sigma point deviations
        self._P = np.zeros((6, 6))
        for i in range(len(sigma_points)):
            residual = sigma_points_pred[i] - self._state
            residual[2] = self._wrap_angle(residual[2])  # Wrap heading residual
            self._P += self._Wc[i] * np.outer(residual, residual)
        
        # Add process noise
        Q = self._process_covariance(dt)
        self._P += Q
        
        # Force symmetry to prevent numerical drift
        self._P = 0.5 * (self._P + self._P.T)
    
    def _motion_model(self, state: np.ndarray, meas: SensorMeasurements, dt: float) -> np.ndarray:
        """
        Apply kinematic motion model to state.
        
        Ackermann steering kinematic model:
            - v_linear from average rear wheel RPM
            - Steering angle constrains heading rate
            - Simple unicycle model with steering coupling
        
        State: [x, y, theta, v_lin, v_ang, gyro_bias]
        """
        x, y, theta, v_lin, v_ang, gyro_bias = state
        
        # Compute linear velocity from rear wheel RPM average
        # RPM → rad/s: rpm / 60 * 2π
        # m/s: rad/s * wheel_radius
        rpm_avg = (meas.rpm_left + meas.rpm_right) / 2.0
        v_lin_meas = (rpm_avg / 60.0 * 2 * math.pi) * self.WHEEL_RADIUS_M
        
        # Ackermann kinematic coupling: steering angle → angular velocity
        # tan(delta) = L * v / R, where R = wheelbase/tan(delta)
        # For small angles: v_angular ≈ v_linear * tan(steering) / wheelbase
        steer_rad = math.radians(meas.steering_angle_deg)
        if abs(v_lin_meas) > 0.01:
            # Kinematic angular velocity from steering
            v_ang_kinematic = v_lin_meas * math.tan(steer_rad) / self.WHEELBASE_M
        else:
            v_ang_kinematic = 0.0
        
        # Smooth linear velocity (use measurement + state blend to reduce noise)
        v_lin_new = 0.7 * v_lin_meas + 0.3 * v_lin
        
        # Smooth angular velocity (kinematic + gyro blend, bias-corrected)
        gyro_z_rad = math.radians(meas.gyro_z_deg_s) - gyro_bias
        v_ang_new = 0.6 * v_ang_kinematic + 0.4 * gyro_z_rad
        
        # Update state via integration
        # Position update (kinematic unicycle)
        x_new = x + v_lin_new * math.cos(theta) * dt
        y_new = y + v_lin_new * math.sin(theta) * dt
        theta_new = theta + v_ang_new * dt
        
        # Bias drift (very slow - gyro bias changes gradually)
        gyro_bias_new = gyro_bias * 0.99  # Exponential decay toward zero
        
        return np.array([x_new, y_new, theta_new, v_lin_new, v_ang_new, gyro_bias_new])
    
    def _update_heading(self, meas: SensorMeasurements) -> None:
        """
        Measurement update from magnetometer heading.
        Uses UKF to fuse compass measurement.
        """
        if meas.mag_heading_deg is None:
            return
        
        # Measurement: heading in radians
        z = math.radians(meas.mag_heading_deg)
        
        # Generate sigma points
        sigma_points = self._generate_sigma_points()
        
        # Extract heading from each sigma point
        heading_sigma = np.array([sp[2] for sp in sigma_points])
        heading_sigma = np.array([self._wrap_angle(h) for h in heading_sigma])
        
        # Mean of predicted heading measurement
        z_pred = np.average(heading_sigma, axis=0, weights=self._Wm)
        z_pred = self._wrap_angle(z_pred)
        
        # Covariance of measurement prediction
        Pzz = 0.0
        Pxz = np.zeros(6)
        for i in range(len(sigma_points)):
            z_residual = self._wrap_angle(heading_sigma[i] - z_pred)
            Pzz += self._Wc[i] * z_residual**2
            state_residual = sigma_points[i] - self._state
            state_residual[2] = self._wrap_angle(state_residual[2])
            Pxz += self._Wc[i] * state_residual * z_residual
        
        # Add measurement noise
        Pzz += self.SIG_HEADING_MAG**2
        
        # Kalman gain
        K = Pxz / Pzz
        
        # Measurement residual (with heading wrapping)
        z_residual = self._wrap_angle(z - z_pred)
        
        # State update
        self._state += K * z_residual
        self._state[2] = self._wrap_angle(self._state[2])
        
        # Update covariance
        self._P -= np.outer(K, Pxz)
    
    def _update_gyro_rate(self, meas: SensorMeasurements) -> None:
        """
        Measurement update from gyro Z-axis.
        Helps correct angular velocity estimate and track gyro bias.
        """
        # Measurement: gyro Z in rad/s
        z_gyro = math.radians(meas.gyro_z_deg_s)
        
        # Predicted gyro measurement is state[4] - state[5]
        # (v_angular - gyro_bias should match measured gyro_z)
        z_pred = self._state[4] - self._state[5]
        
        # Simple scalar update (no full UKF for 1-D measurement)
        innovation = z_gyro - z_pred
        
        # Innovation covariance
        S = self._P[4, 4] + self._P[5, 5] - 2*self._P[4, 5] + self.SIG_GYRO_Z**2
        
        # Kalman gain (only update v_angular and gyro_bias)
        K_v = self._P[4, 4] - self._P[4, 5]
        K_b = self._P[5, 4] - self._P[5, 5]
        
        if S > 0:
            K_v /= S
            K_b /= S
            
            self._state[4] += K_v * innovation  # Update v_angular
            self._state[5] += K_b * innovation  # Update gyro_bias
            
            # Covariance update (simplified)
            self._P[4, 4] -= K_v * self._P[4, 4]
            self._P[5, 5] -= K_b * self._P[5, 5]
            self._P[4, 5] -= K_v * self._P[4, 5]
            self._P[5, 4] -= K_b * self._P[5, 4]
    
    def _generate_sigma_points(self) -> np.ndarray:
        """Generate sigma points for UKF."""
        n = 6
        sigma_points = np.zeros((2*n + 1, n))
        sigma_points[0] = self._state
        
        # Compute Cholesky decomposition of covariance
        try:
            L = np.linalg.cholesky(self._P * (n + self._lambda))
        except np.linalg.LinAlgError:
            # If Cholesky fails, force P to be positive-definite via eigenvalue repair
            scaled = self._P * (n + self._lambda)
            scaled = 0.5 * (scaled + scaled.T)  # enforce symmetry
            eigvals, eigvecs = np.linalg.eigh(scaled)
            eigvals = np.maximum(eigvals, 1e-6)  # clamp negative eigenvalues
            repaired = eigvecs @ np.diag(eigvals) @ eigvecs.T
            L = np.linalg.cholesky(repaired)
        
        for i in range(n):
            sigma_points[1 + i] = self._state + L[i]
            sigma_points[1 + n + i] = self._state - L[i]
        
        return sigma_points
    
    def _process_covariance(self, dt: float) -> np.ndarray:
        """Process (motion) noise covariance matrix."""
        Q = np.eye(6)
        Q[0, 0] = 0  # x position doesn't add noise directly
        Q[1, 1] = 0  # y position doesn't add noise directly
        Q[2, 2] = (self.SIG_V_ANGULAR * dt)**2  # theta accumulates angular velocity noise
        Q[3, 3] = (self.SIG_V_LINEAR)**2        # linear velocity noise
        Q[4, 4] = (self.SIG_V_ANGULAR)**2        # angular velocity noise
        Q[5, 5] = (self.SIG_GYRO_BIAS)**2        # gyro bias drift
        return Q
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π] radians."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_state(self) -> RoverOdometryState:
        """Get current pose and velocity estimate."""
        return RoverOdometryState(
            x=self._state[0],
            y=self._state[1],
            theta=self._state[2],
            v_linear=self._state[3],
            v_angular=self._state[4],
            timestamp_s=self._last_time_s,
        )
    
    def get_covariance(self) -> np.ndarray:
        """Get state covariance matrix (6×6)."""
        return self._P.copy()
    
    def get_position(self) -> Tuple[float, float]:
        """Get (x, y) position estimate."""
        return (self._state[0], self._state[1])
    
    def get_heading(self) -> float:
        """Get heading estimate in radians."""
        return self._state[2]
    
    def get_heading_deg(self) -> float:
        """Get heading estimate in degrees."""
        return math.degrees(self._state[2])
    
    def get_linear_velocity(self) -> float:
        """Get linear velocity estimate (m/s)."""
        return self._state[3]
    
    def get_angular_velocity(self) -> float:
        """Get angular velocity estimate (rad/s)."""
        return self._state[4]
    
    def get_gyro_bias(self) -> float:
        """Get estimated gyro Z bias (rad/s)."""
        return self._state[5]
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Reset pose estimate to known position."""
        self._state.fill(0.0)
        self._state[0] = x
        self._state[1] = y
        self._state[2] = theta
        self._P = np.eye(6)
        self._P[2, 2] = math.radians(5)**2  # Small heading uncertainty after reset
        self._last_time_s = time.time()
    
    def set_uncertainty(self, pos_std_m: float = 0.05, heading_std_deg: float = 5.0) -> None:
        """Increase uncertainty (e.g., after wheel slip detected)."""
        self._P[0, 0] += pos_std_m**2
        self._P[1, 1] += pos_std_m**2
        self._P[2, 2] += math.radians(heading_std_deg)**2


# Convenience function for integration
def create_odometry_measurement(
    timestamp_s: float,
    rpm_left: float,
    rpm_right: float,
    gyro_z_deg_s: float,
    mag_heading_deg: Optional[float] = None,
    steering_angle_deg: float = 0.0,
    accel_x: float = 0.0,
) -> SensorMeasurements:
    """
    Factory function to create a sensor measurement.
    
    Typically called from main loop:
        meas = create_odometry_measurement(
            timestamp_s=time.time(),
            rpm_left=sensor_packet.rpm_left,
            rpm_right=sensor_packet.rpm_right,
            gyro_z_deg_s=sensor_packet.gyro_z,
            mag_heading_deg=compute_heading_from_mag(...),
            steering_angle_deg=current_steering_cmd,
        )
        odometry.update(meas)
    """
    return SensorMeasurements(
        timestamp_s=timestamp_s,
        rpm_left=rpm_left,
        rpm_right=rpm_right,
        gyro_z_deg_s=gyro_z_deg_s,
        mag_heading_deg=mag_heading_deg,
        steering_angle_deg=steering_angle_deg,
        accel_x=accel_x,
    )
