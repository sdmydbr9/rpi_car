# Performance Analysis: Slow RPM Updates in Cockpit UI

## Problem Summary
- **Pico sensor reader**: Delivers instant, real-time RPM data (50Hz from sensors)
- **Flask/SocketIO backend**: Updates car_state RPM at 50Hz, broadcasts rpm_update at 50Hz
- **Cockpit UI**: Shows RPM data that appears to be **100+ seconds old** (stale data issue)
- **RPM limits not respected**: Car drives beyond gear limits because control system receives old RPM feedback

## Root Causes Identified

### 1. **Data Flow Bottleneck in `telemetry_broadcast()`** [CRITICAL]
**Location**: [scripts/main.py](scripts/main.py#L6370)
**Issue**: The telemetry_broadcast thread emits at 20Hz (50ms intervals) but reads FROM `car_state["encoder_rpm_left/right"]` which may not always reflect the LATEST Pico packet.
```python
telemetry_data = {
    "rpm": real_rpm,           # reads from car_state (potentially stale)
    "rpm_left": rpm_left,      # reads from car_state (not fresh Pico data)
    "rpm_right": rpm_right,    # reads from car_state (not fresh Pico data)
    ...
}
socketio.emit('telemetry_update', telemetry_data)
time.sleep(0.05)  # 50ms = 20Hz
```

**Problem**: If `encoder_and_power_thread` (50Hz) gets blocked or delayed at any point, `telemetry_broadcast` will emit outdated RPM values that were cached in `car_state` from seconds ago.

### 2. **UI Uses Stale Telemetry When rpm_update Delays** [CRITICAL]
**Location**: [src/components/cockpit/CockpitController.tsx](src/components/cockpit/CockpitController.tsx#L720-L725)
```tsx
// RPM is updated by the dedicated 50Hz rpm_update stream so the slower
// telemetry packet does not overwrite fresher gauge values.
```

**Problem**: The logic assumes `rpm_update` (50Hz) is ALWAYS more recent than `telemetry_update` (20Hz). However:
- If the 50Hz rpm_update stream has ANY delay or gap, the UI might fall back to stale telemetry data
- There's no freshness timestamp to validate data age
- Telemetry update can overwrite if it arrives between rpm_update messages

### 3. **No Per-Packet Timestamps in Socket Events**
**Location**: [scripts/main.py](scripts/main.py#L6351) and [src/lib/socketClient.ts](src/lib/socketClient.ts#L132)
**Issue**: Both `rpm_update` and `telemetry_update` emit data WITHOUT packet timestamps. The UI cannot detect data staleness.

```python
# NO TIMESTAMP INCLUDED
socketio.emit('rpm_update', {
    "rpm": rpm_avg,
    "rpm_left": rpm_left,
    "rpm_right": rpm_right,
    "target_rpm": round(float(car_state.get("current_target_rpm", 0.0) or 0.0), 1),
    "engine_running": car_state["engine_running"],
})
```

### 4. **Pico UART Packet Queue May Be Delayed**
**Location**: [scripts/core/pico_sensor_reader.py](scripts/core/pico_sensor_reader.py#L201-L228)
**Issue**: The reader uses a background thread with a circular buffer. If the main control loop (50Hz) doesn't call `get_sensor_packet()` frequently enough, it may read packets that are SEVERAL frames old.

```python
def _read_loop(self):
    """Background thread: continuously read and parse JSON packets."""
    while self._running and self._serial:
        try:
            line = self._serial.readline().decode().strip()
            # ... parse packet
            with self._lock:
                self._last_packet = packet  # Only keeps ONE packet
```

**Problem**: The reader only keeps the LATEST packet. If the control loop lags, it might read a packet that's 100+ ms old and think it's fresh.

### 5. **Control Loop May Be Below 50Hz Due to Other Threads**
**Location**: [scripts/main.py](scripts/main.py#L6178-6196)
**Issue**: The `encoder_and_power_thread` is designed for 50Hz but can be deprioritized by the OS:
- Heavy camera processing may steal CPU cycles
- Flask request handling can block threads
- Other I/O operations (UART, GPIO) may cause jitter
- No realtime scheduling is used (not running with `SCHED_FIFO`)

## Secondary Issues

### 6. **UI Rendering Lag Not Accounted For**
**Location**: [src/components/cockpit/CarTelemetry.tsx](src/components/cockpit/CarTelemetry.tsx#L502-L515)
**Issue**: Even with fresh data, React re-renders can be delayed by:
- Bundle size / JS parse/exec time
- CSS recalculations for gauge animations
- Re-rendering large telemetry panels on every update
- No virtualization or memoization of sub-components

### 7. **Wheel Sync / Motor Control Decoupled from Fresh RPM**
**Location**: [scripts/core/wheel_sync.py](scripts/core/wheel_sync.py#L358-L380)
**Issue**: The motor control depends on `get_rpm()` callback, which may read stale car_state if the encoder_and_power_thread hasn't updated in time:
```python
def _correct_impl(self, speed_l, speed_r, ...):
    rpm = self.get_rpm()  # May be old if encoder_and_power_thread hasn't run
    # Clamp target RPMs to gear limits using potentially stale RPM feedback
    rpm_fl = rpm.get('front_left', 0.0)
```

**Result**: RPM limits aren't enforced correctly because the PID control loop receives outdated speed feedback.

## Why Data Appears "100+ Seconds Old"

**Hypothesis**: There's likely a rare edge case where:
1. `encoder_and_power_thread` gets preempted and doesn't update for 50-100ms windows
2. Multiple consecutive telemetry_broadcast emissions use the same stale `car_state` values
3. Telemetry packets accumulate on the network/WebSocket queue
4. UI processes them in batches, displaying data from 50-100 iterations ago
5. At 20Hz telemetry, 100 iterations = **5 seconds**. At 50ms intervals: 100 × 0.05 = **5+ seconds**. If there's queue buildup over time, it could compound to 100+ seconds.

**Alternative cause**: The pico UART reader itself might have buffering delays where packets sit in the OS serial buffer for extended periods before being read.

## Recommended Changes

### PRIORITY 1: Add Packet Timestamps
Add millisecond timestamps to all socket events so UI can detect stale data:
```python
import time
socketio.emit('rpm_update', {
    "rpm": rpm_avg,
    "rpm_left": rpm_left,
    "rpm_right": rpm_right,
    "target_rpm": target_rpm,
    "engine_running": car_state["engine_running"],
    "timestamp_ms": int(time.time() * 1000),  # Add this
})
```

### PRIORITY 2: Decouple Backend Motor Control from UI
The motor control (wheel_sync, RPM PID) should NOT wait for telemetry broadcast cycles. Instead:
- Create a dedicated **Fast Control Loop** (100+ Hz) that runs independent of Flask/WebSocket
- This loop reads fresh Pico data directly and updates motor PWM without waiting for UI broadcasts
- Keep telemetry_broadcast as display-only (non-critical path)

### PRIORITY 3: Fix Data Freshness in UI
Implement freshness checking:
```tsx
const freshThresholdMs = 100; // 100ms tolerance
const isStaleData = (Date.now() - packetTimestamp) > freshThresholdMs;
if (isStaleData) {
    // Display warning or skip this update
}
```

### PRIORITY 4: Add Realtime Scheduling
Run the high-priority control threads with `SCHED_FIFO` priority:
```bash
sudo chrt -f 50 python main.py  # Run at FIFO priority 50
```

### PRIORITY 5: Optimize Pico UART Reading
Ensure serial buffer doesn't accumulate delays:
- Set serial port to non-blocking mode
- Add buffer overflow detection
- Log packet read latencies

### PRIORITY 6: Separate Control Loop from Telemetry Loop
```python
# Current architecture (coupled):
encoder_and_power_thread  →  car_state  ←  telemetry_broadcast  →  UI

# Proposed architecture (decoupled):
encoder_and_power_thread  →  (DIRECT to motor_control, ALSO updates car_state)
telemetry_broadcast (20Hz)  →  (reads car_state for display, lower priority)
UI  ←  rpm_update (50Hz) + telemetry_update (20Hz)
```

## Summary Table

| Component | Frequency | Latency Issue | Impact |
|-----------|-----------|---------------|--------|
| Pico Sensor | ~50Hz | May have buffering delays | Data starts old |
| encoder_and_power_thread | 50Hz (target) | Subject to OS preemption | car_state updates delayed |
| rpm_broadcast | 50Hz | Depends on encoder_thread freshness | UI RPM may be stale |
| telemetry_broadcast | 20Hz | Uses cached car_state values | Compounding delays |
| UI RPM display | Event-driven | React render lag added | Final display is delayed |
| Motor control (wheel_sync) | ~100Hz (internal) | Depends on get_rpm() freshness | RPM limits not enforced |

## Key Insight
**The problem is architectural, not just performance**: The current system couples the *critical motor control path* to the *non-critical telemetry/display path*. When telemetry gets backed up (due to network, rendering, or thread preemption), it indirectly starves the motor controller of fresh RPM feedback, which then allows the car to ignore RPM limits.

The fix requires separating these into independent paths:
- **Fast/Critical**: Pico → Motor Control → Stay within limits
- **Slow/Non-Critical**: Pico → Car State → Telemetry → UI (for display only)
