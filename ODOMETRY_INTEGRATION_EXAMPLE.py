"""
Example Integration of Fused Odometry into main.py

This shows EXACTLY where and how to integrate the odometry system
into your existing rover control code.

Search for "ODOMETRY INTEGRATION EXAMPLE" in this file to see the changes.
"""

# ===============================================================================
# IMPORTS — Add to top of scripts/main.py
# ===============================================================================

# Existing imports...
from pico_sensor_reader import (
    init_pico_reader, get_gyro_z as pico_get_gyro_z,
    get_accel_xyz as pico_get_accel_xyz,
    get_laser_distance_cm as pico_get_laser_cm,
    get_laser_distance_mm as pico_get_laser_mm,
    get_battery_voltage as pico_get_battery_voltage,
    get_current_sense as pico_get_current_sense,
    get_sensor_packet as pico_get_sensor_packet,
)

# ===== ODOMETRY INTEGRATION EXAMPLE #1: IMPORT =====
from odometry_integration import (
    init_odometry,
    update_odometry,
    get_diagnostics_dict,
    get_position,
    get_heading_deg,
    get_linear_velocity,
    get_angular_velocity,
)
from compass_calibration import load_calibration, compute_heading_degrees


# ===============================================================================
# INITIALIZATION — In the main() function, after pico_reader init
# ===============================================================================

def main():
    """Main entry point."""
    
    # ... existing initialization code ...
    
    print("🔧 [System] Initializing Pico sensor reader...")
    init_pico_reader()
    
    # ===== ODOMETRY INTEGRATION EXAMPLE #2: INIT =====
    print("🔧 [System] Initializing fused odometry...")
    init_odometry()
    
    # Load compass calibration
    try:
        compass_cal = load_calibration('scripts/diagnostics/compass_cal.json')
        print("✅ [Compass] Calibration loaded")
    except Exception as e:
        compass_cal = None
        print(f"⚠️  [Compass] Calibration not found: {e}")
    
    # ... rest of initialization ...


# ===============================================================================
# SENSOR LOOP — In the main sensor reading loop (~50 Hz)
# ===============================================================================

def process_sensor_cycle():
    """
    Main sensor processing cycle, called ~50 Hz/60 Hz.
    Existing code processes encoders, IMU, etc.
    
    This is where you'd integrate odometry update.
    """
    
    # Get sensor packet from Pico
    sensor_packet = pico_get_sensor_packet()
    if not sensor_packet:
        return
    
    # Existing processing...
    # encoder_left = sensor_packet.enc_left_steps
    # encoder_right = sensor_packet.enc_right_steps
    # rpm_left = sensor_packet.rpm_left
    # rpm_right = sensor_packet.rpm_right
    # accel_x, accel_y, accel_z = sensor_packet.accel_x, ...
    # gyro_x, gyro_y, gyro_z = sensor_packet.gyro_x, ...
    
    # ===== ODOMETRY INTEGRATION EXAMPLE #3: UPDATE IN SENSOR LOOP =====
    # Compute magnetometer heading from calibrated magnetometer data
    mag_heading_deg = None
    mag_quality = 0.0
    if compass_cal is not None:
        try:
            mag_heading_deg, mag_quality = compute_heading_degrees(
                sensor_packet.mag_x,
                sensor_packet.mag_y,
                sensor_packet.mag_z,
                compass_cal
            )
        except Exception as e:
            # Compass computation failed, skip this cycle
            pass
    
    # Get current steering angle command
    # (You need to track this in your steering control code)
    current_steering_angle_deg = car_state.get("steer_angle", 0.0)
    
    # Update odometry with latest sensor measurements
    odometry_state = update_odometry(
        rpm_left=sensor_packet.rpm_left,
        rpm_right=sensor_packet.rpm_right,
        gyro_z_deg_s=sensor_packet.gyro_z,
        mag_heading_deg=mag_heading_deg,
        steering_angle_deg=current_steering_angle_deg,
        accel_x=sensor_packet.accel_x,
    )
    
    if odometry_state:
        # Store in car_state for WebSocket emission
        car_state["odometry_x_m"] = odometry_state.x
        car_state["odometry_y_m"] = odometry_state.y
        car_state["odometry_heading_deg"] = math.degrees(odometry_state.theta)
        car_state["odometry_v_linear_m_s"] = odometry_state.v_linear
        car_state["odometry_v_angular_rad_s"] = odometry_state.v_angular


# ===============================================================================
# AUTONOMOUS CONTROL — Example use in autopilot
# ===============================================================================

class AutoPilotController:
    """Example of using fused odometry in autonomous control."""
    
    def compute_steering(self, target_heading_deg: float) -> float:
        """
        Compute steering angle to reach target heading.
        Uses FUSED heading instead of compass/gyro alone.
        """
        # Get fused heading estimate
        current_heading = get_heading_deg()
        
        # Compute heading error
        error = current_heading - target_heading_deg
        # Wrap to [-180, 180)
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # PID control
        steering_angle = self._pid_controller.update(error)
        steering_angle = max(-35, min(35, steering_angle))  # Clamp
        
        return steering_angle
    
    def compute_speed_ramp(self, target_distance_m: float) -> float:
        """
        Compute speed to travel target distance.
        Uses FUSED linear velocity, not just encoder RPM.
        """
        x, y = get_position()
        current_distance = math.sqrt(x**2 + y**2)
        
        remaining = target_distance_m - current_distance
        
        # Proportional speed control
        if remaining < 0.1:
            # Near target, slow down
            return 30  # PWM%
        elif remaining < 0.5:
            # Coasting zone
            return 50
        else:
            # Full speed
            return 80
    
    def get_odometry_for_slam(self) -> dict:
        """Export fused odometry for SLAM backend."""
        diagnostics = get_diagnostics_dict()
        return {
            "pose": {
                "x": diagnostics["x_m"],
                "y": diagnostics["y_m"],
                "theta": diagnostics["heading_rad"],
            },
            "velocity": {
                "linear": diagnostics["v_linear_m_s"],
                "angular": diagnostics["v_angular_rad_s"],
            },
            "covariance": {
                "position": diagnostics["std_pos_x_m"],
                "heading": diagnostics["std_heading_deg"],
            },
            "timestamp": diagnostics["timestamp_s"],
        }


# ===============================================================================
# WEBSOCKET EMISSION — Broadcast odometry to frontend
# ===============================================================================

def broadcast_telemetry():
    """
    Broadcast rover telemetry via WebSocket.
    Include odometry state in the telemetry payload.
    """
    diagnostics = get_diagnostics_dict()
    
    telemetry = {
        # ... existing telemetry ...
        
        # ===== ODOMETRY INTEGRATION EXAMPLE #4: TELEMETRY =====
        "odometry": {
            "position": {
                "x_m": diagnostics["x_m"],
                "y_m": diagnostics["y_m"],
            },
            "heading_deg": diagnostics["heading_deg"],
            "velocity": {
                "linear_m_s": diagnostics["v_linear_m_s"],
                "angular_rad_s": diagnostics["v_angular_rad_s"],
            },
            "uncertainty": {
                "position_m": diagnostics["std_pos_x_m"],
                "heading_deg": diagnostics["std_heading_deg"],
            },
            "timestamp_s": diagnostics["timestamp_s"],
        },
    }
    
    socketio.emit('telemetry', telemetry)


# ===============================================================================
# DEBUGGING — Print odometry diagnostics
# ===============================================================================

@socketio.on('request_odometry_debug')
def handle_odometry_debug(data):
    """SocketIO handler: return detailed odometry diagnostics."""
    diagnostics = get_diagnostics_dict()
    
    response = f"""
    🧭 FUSED ODOMETRY DIAGNOSTICS
    ════════════════════════════════
    Position:
      X: {diagnostics['x_m']:.3f} m (±{diagnostics['std_pos_x_m']:.3f} m)
      Y: {diagnostics['y_m']:.3f} m (±{diagnostics['std_pos_y_m']:.3f} m)
    
    Orientation:
      Heading: {diagnostics['heading_deg']:.1f}° (±{diagnostics['std_heading_deg']:.1f}°)
    
    Velocity:
      Linear: {diagnostics['v_linear_m_s']:.3f} m/s (±{diagnostics['std_v_linear']:.3f})
      Angular: {math.degrees(diagnostics['v_angular_rad_s']):.1f}°/s (±{math.degrees(diagnostics['std_v_angular']):.1f}°/s)
    
    Filter State:
      Gyro bias: {diagnostics['gyro_bias_rad_s']:.6f} rad/s
      Updated: {time.time() - diagnostics['timestamp_s']:.2f} sec ago
    """
    
    print(response)
    emit('odometry_debug', {'diagnostics': response})


# ===============================================================================
# EXPORT FOR EXTERNAL SYSTEMS (e.g., ROS via HTTP)
# ===============================================================================

@app.route('/api/odometry')
def api_get_odometry():
    """HTTP endpoint: return current odometry as JSON."""
    diagnostics = get_diagnostics_dict()
    
    return jsonify({
        "status": "ok",
        "data": {
            "position": {
                "x_m": diagnostics["x_m"],
                "y_m": diagnostics["y_m"],
            },
            "heading": {
                "deg": diagnostics["heading_deg"],
                "rad": diagnostics["heading_rad"],
            },
            "velocity": {
                "linear_m_s": diagnostics["v_linear_m_s"],
                "angular_rad_s": diagnostics["v_angular_rad_s"],
            },
            "uncertainty": {
                "position_std_m": diagnostics["std_pos_x_m"],
                "heading_std_deg": diagnostics["std_heading_deg"],
            },
            "timestamp_s": diagnostics["timestamp_s"],
        }
    })


# ===============================================================================
# SUMMARY
# ===============================================================================

"""
INTEGRATION CHECKLIST:

1. ✅ Import statements (Example #1)
   - Add: from odometry_integration import ...
   - Add: from compass_calibration import load_calibration, compute_heading_degrees

2. ✅ Initialization (Example #2)
   - Call init_odometry() after init_pico_reader()
   - Load compass calibration

3. ✅ Update in sensor loop (Example #3)
   - Call update_odometry() with current sensor values
   - Store results in car_state

4. ✅ Use in autonomous control (Example section above)
   - Call get_heading_deg(), get_linear_velocity(), etc. in autopilot

5. ✅ Broadcast to frontend (Example #4)
   - Emit odometry in telemetry via socketio

6. ✅ Debug endpoints
   - Add SocketIO handler for odometry diagnostics
   - Add HTTP endpoint for externals systems (ROS, etc.)

VARIABLE TRACKING:
- Must track current_steering_angle_deg somewhere (in SteeringController)
- Must have compass_cal loaded at module level for update loop access

TEST:
1. Start rover: python3 scripts/main.py
2. Check console: Should see "✅ [Odometry] Fused odometry system initialized"
3. Drive in circle: Monitor /api/odometry endpoint — heading should change
4. Check uncertainty: Should decrease as time goes on (Kalman filter converging)
"""
