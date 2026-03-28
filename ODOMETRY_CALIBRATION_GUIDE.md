# Fused Odometry — Calibration & Parameter Tuning Guide

## Overview

The rover uses an **Unscented Kalman Filter (UKF)** to fuse rear wheel encoders, MPU6500 gyroscope, QMC5883L magnetometer, and Ackermann steering geometry into a single pose estimate (x, y, heading, velocity).

---

## 1. Prerequisites

| Sensor | Where | What it Provides |
|--------|-------|-----------------|
| Rear encoders (L/R) | Pico W (UART @ 50 Hz) | Wheel RPM → linear velocity |
| MPU6500 gyro Z | Pico W (UART @ 50 Hz) | Yaw rate (°/s) |
| QMC5883L magnetometer | Pico W (UART @ 50 Hz) | Absolute heading (°) |
| Steering servo | PWM from Pi | Front wheel angle (°) |

All sensor data flows through `pico_sensor_reader` → `car_state` → `update_odometry()` at **20 Hz** (encoder/power thread).

---

## 2. Compass Calibration (Critical First Step)

The magnetometer MUST be calibrated or heading measurements will corrupt the filter.

### Run the calibration utility

```bash
cd ~/rpi_car/scripts
python3 -m diagnostics.compass_calibration
```

Follow the on-screen instructions to rotate the rover through 360° in all three axes. The tool saves calibration offsets/scales to `scripts/diagnostics/compass_cal.json`.

### Verify calibration

```bash
python3 -m diagnostics.test_drive
```

Watch the compass heading output — it should smoothly track 0–360° as you rotate the rover.

### Troubleshooting
- **Heading jumps/wraps**: Re-run calibration. Ensure no magnets or motors are energized during cal.
- **Always reads ~0°**: Check QMC5883L wiring (I2C address 0x0D on Pico).
- **Noisy heading**: Increase `SIG_HEADING_MAG` in `fused_odometry.py` (default: 5°). Try 8–10° if readings are very noisy.

---

## 3. UKF Parameter Tuning

All parameters are constants in `scripts/core/fused_odometry.py` class `FusedOdometry`.

### Process Noise (Q matrix — how much we trust the motion model)

| Parameter | Default | Units | What it controls |
|-----------|---------|-------|-----------------|
| `SIG_V_LINEAR` | 0.15 | m/s | Encoder velocity trust. **Increase** if wheels slip (grass, gravel). **Decrease** on smooth floors. |
| `SIG_V_ANGULAR` | 0.10 | rad/s | Angular velocity trust from steering model. **Increase** if Ackermann geometry is imprecise. |
| `SIG_GYRO_BIAS` | 0.001 | rad/s | How fast gyro bias drifts. Default is conservative — only increase if gyro drifts badly. |

**Rule of thumb**: Higher process noise = filter relies MORE on measurements, LESS on motion model. If the rover's actual path overshoots the estimated path, increase process noise.

### Measurement Noise (R matrix — how much we trust sensors)

| Parameter | Default | Units | What it controls |
|-----------|---------|-------|-----------------|
| `SIG_HEADING_MAG` | 5.0° | rad (stored as radians) | Magnetometer heading trust. **Increase** near metal/motors (10–15°). **Decrease** in open areas (3°). |
| `SIG_GYRO_Z` | 1.0° | rad (stored as radians) | Gyro yaw rate trust. MPU6500 is well-calibrated; 1° is a good starting point. |

**Rule of thumb**: Higher measurement noise = filter relies MORE on motion model, LESS on that sensor.

### Rover Geometry (must match physical hardware)

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `WHEELBASE_M` | 0.210 | m | Rear axle to front steering axle. Measure carefully. |
| `WHEEL_RADIUS_M` | 0.0375 | m | 75mm diameter wheels / 2. |
| `TRACK_WIDTH_M` | 0.172 | m | Distance between rear wheel centers. |

If position drifts during turns but straight-line is accurate, **measure the wheelbase and track width again** — these are the most impactful geometry parameters.

---

## 4. Tuning Workflow

### Step 1: Straight-line test

1. Place rover at a known start point
2. Drive forward ~2 meters in a straight line
3. Check odometry via: `curl http://localhost:5000/api/odometry`
4. Compare `x_m`/`y_m` to actual displacement

**If too short**: Wheel radius may be wrong, or encoder PPR needs correction.  
**If drifting sideways**: Compass calibration is off, or `SIG_HEADING_MAG` is too low (trusting bad heading).

### Step 2: Rotation test

1. Reset odometry: `curl -X POST http://localhost:5000/api/odometry/reset`
2. Rotate rover 360° in place
3. Check heading returns to ~0°
4. Check x/y stays near (0, 0)

**If heading doesn't return to 0°**: Gyro bias tracking may need help. Increase `SIG_GYRO_BIAS` to 0.005.  
**If x/y drifts during rotation**: Track width is wrong, or encoder differential is miscalibrated.

### Step 3: Square test

1. Reset odometry
2. Drive a ~1m square (forward, right 90°, forward, right 90°, ... )
3. Check rover returns close to origin

**Cumulative heading drift**: Increase compass weight (decrease `SIG_HEADING_MAG`).  
**Position but not heading drift**: Increase `SIG_V_LINEAR`.

---

## 5. API Reference

### HTTP Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/odometry` | GET | Full odometry state (position, velocity, uncertainties) |
| `/api/odometry/reset` | POST | Reset pose. Optional JSON body: `{"x": 0, "y": 0, "heading_deg": 0}` |

### SocketIO Telemetry (20 Hz via `telemetry_update` event)

Fields added to the existing telemetry payload:

| Field | Type | Description |
|-------|------|-------------|
| `odometry_x_m` | float | X position (meters, East) |
| `odometry_y_m` | float | Y position (meters, North) |
| `odometry_heading_deg` | float | Heading (degrees) |
| `odometry_v_linear` | float | Linear velocity (m/s) |
| `odometry_v_angular` | float | Angular velocity (rad/s) |
| `odometry_active` | bool | True when odometry is running |

### Python API (from `odometry_integration`)

```python
from odometry_integration import (
    init_odometry,       # Call once at startup
    update_odometry,     # Call each sensor cycle (~20 Hz)
    get_position,        # → (x_m, y_m)
    get_heading_deg,     # → float (degrees)
    get_linear_velocity, # → float (m/s)
    get_angular_velocity,# → float (rad/s)
    get_diagnostics_dict,# → full state dict
    reset_pose,          # Reset to (x, y, heading_deg)
    increase_uncertainty,# Inject uncertainty after slip events
)
```

---

## 6. Runtime Diagnostics

### Check odometry is running
```bash
curl -s http://localhost:5000/api/odometry | python3 -m json.tool
```

Expected output:
```json
{
    "active": true,
    "x_m": 0.0,
    "y_m": 0.0,
    "heading_deg": 45.2,
    "v_linear_m_s": 0.0,
    "std_pos_x_m": 0.01,
    "std_heading_deg": 2.3,
    ...
}
```

### Monitor position in real-time
```bash
watch -n 0.5 'curl -s http://localhost:5000/api/odometry | python3 -m json.tool'
```

### Reset position before a test run
```bash
curl -X POST http://localhost:5000/api/odometry/reset \
  -H "Content-Type: application/json" \
  -d '{"x": 0, "y": 0, "heading_deg": 0}'
```

---

## 7. Common Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `"active": false` | `init_odometry()` failed or never called | Check startup logs for "⚠️ Fused odometry initialization error" |
| Position grows when stationary | Encoder noise / IMU vibration | Check RPM reads 0 when stopped; ensure rover is on flat surface |
| Heading drifts slowly | Gyro bias not converging | Drive in circles to exercise compass correction; increase `SIG_GYRO_BIAS` |
| Position way off after turns | Wrong wheelbase/track | Measure physical geometry; update constants |
| Compass heading wrong after motor start | EM interference from motors | Increase `SIG_HEADING_MAG` to 10–15° to reduce compass influence |
| `numpy` import error | Missing dependency | `pip3 install numpy` |
