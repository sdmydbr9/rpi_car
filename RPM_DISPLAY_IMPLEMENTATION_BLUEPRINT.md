# RPM Display & Motor Control Issues - Implementation Blueprint

## Quick Summary

Your system has an **architectural bottleneck** where motor control and telemetry display are coupled through a shared dictionary. When telemetry broadcast gets delayed (which happens easily due to network queueing, React rendering, or OS scheduling), motor control receives stale RPM feedback, which allows the car to:
1. Display outdated RPM values in the UI (100+ seconds old)
2. Ignore RPM limits configured for each gear

## The 3 Key Problems

### Problem 1: No Data Timestamps
**Current**: Socket events have no timestamps
```json
{ "rpm": 45.3, "rpm_left": 44.0, "rpm_right": 46.5 }
```
**Result**: UI can't tell if data is fresh or 100 seconds old

**Fix**: Add millisecond timestamp
```json
{ "rpm": 45.3, "rpm_left": 44.0, "rpm_right": 46.5, "timestamp_ms": 1712245678900 }
```

### Problem 2: Coupled Control & Display Paths
**Current**:
```
Pico (50Hz) → encoder_and_power_thread (50Hz) → car_state dict 
                                                    ↓
                                    telemetry_broadcast (20Hz) 
                                                    ↓
                                            Network + UI render
                                                    ↓
                            Motor control ALSO reads from car_state
                            (so it gets telemetry delays too!)
```

**Result**: Motor control waits for telemetry, ignores RPM limits

**Fix**: Create independent fast control loop
```
Pico (50Hz) → Fast Control Loop (100+ Hz, SCHED_FIFO priority)
              ↓                     ↓
        Motor Control         car_state (non-critical)
        (respects limits)            ↓
                           telemetry_broadcast (20Hz)
```

### Problem 3: Variable Thread Delays
**Current**: encoder_and_power_thread targets 50Hz but:
- Can be preempted by OS
- Loses priority to camera processing
- No guarantee of 20ms execution

**Result**: RPM updates not consistent, car_state gets stale

**Fix**: Use realtime scheduling (SCHED_FIFO) for critical threads

## Specific Changes Needed

### Change 1: Add Timestamps to All Socket Events
**Files**: `scripts/main.py`

**Location 1** - rpm_broadcast function (~line 6351):
```python
# BEFORE:
socketio.emit('rpm_update', {
    "rpm": rpm_avg,
    "rpm_left": rpm_left,
    "rpm_right": rpm_right,
    "target_rpm": round(...),
    "engine_running": car_state["engine_running"],
})

# AFTER (add timestamp):
socketio.emit('rpm_update', {
    "rpm": rpm_avg,
    "rpm_left": rpm_left,
    "rpm_right": rpm_right,
    "target_rpm": round(...),
    "engine_running": car_state["engine_running"],
    "timestamp_ms": int(time.time() * 1000),  # Add this line
})
```

**Location 2** - telemetry_broadcast function (~line 6410):
```python
# Before socketio.emit('telemetry_update', telemetry_data):
telemetry_data["timestamp_ms"] = int(time.time() * 1000)
socketio.emit('telemetry_update', telemetry_data)
```

### Change 2: Add Freshness Validation in UI
**Files**: `src/lib/socketClient.ts`, `src/components/cockpit/CockpitController.tsx`

**Update RpmUpdateData interface**:
```typescript
export interface RpmUpdateData {
  rpm: number;
  rpm_left?: number;
  rpm_right?: number;
  target_rpm?: number;
  engine_running?: boolean;
  timestamp_ms?: number;  // Add this
}
```

**Add freshness check in CockpitController**:
```tsx
socketClient.onRpmUpdate((data) => {
  const now = Date.now();
  const ageMs = data.timestamp_ms ? now - data.timestamp_ms : 0;
  const isStale = ageMs > 200; // 200ms = stale
  
  if (!isStale) {  // Only update if fresh
    setControlState(prev => ({
      ...prev,
      rpm: data.engine_running ? (data.rpm ?? prev.rpm) : 0,
      rpmLeft: data.engine_running ? (data.rpm_left ?? prev.rpmLeft) : 0,
      rpmRight: data.engine_running ? (data.rpm_right ?? prev.rpmRight) : 0,
      targetRpm: data.engine_running ? (data.target_rpm ?? prev.targetRpm) : 0,
    }));
  }
});
```

### Change 3: Create Independent Fast Control Loop
**Files**: `scripts/main.py`, `scripts/core/motor.py`

**Create new function** (~after encoder_and_power_thread):
```python
def fast_motor_control_loop():
    """
    CRITICAL PATH (100+ Hz, SCHED_FIFO priority)
    Directly reads fresh Pico data and controls motors.
    Independent of telemetry/UI delays.
    """
    interval = 0.01  # 10ms = 100Hz
    
    while True:
        try:
            time.sleep(interval)
            
            # Read FRESH Pico packet directly (not from car_state)
            pico_pkt = pico_get_sensor_packet()
            if pico_pkt is None:
                continue
            
            # Get current target RPM from car_state (updated by UI/gamepad)
            target_rpm = car_state.get("current_target_rpm", 0.0)
            
            # Call motor PID with fresh RPM data
            car_system.rpm_pid_tick(target_rpm)
            
            # Apply motor commands
            pwm_l, pwm_r = car_system.get_pwm()
            send_lr_pwm(int(pwm_l), int(pwm_r))
            
        except Exception as e:
            print(f"❌ Fast motor control error: {e}")
            time.sleep(interval)

# Start with high priority
motor_thread = threading.Thread(target=fast_motor_control_loop, daemon=True)
motor_thread.start()
print("🚀 [Control] Fast motor control loop started (100Hz, SCHED_FIFO)")
```

**Wrap with realtime priority** (after thread start):
```python
import os
import signal

def set_realtime_priority(pid, priority=50):
    """Set SCHED_FIFO realtime priority for a process."""
    try:
        os.sched_setscheduler(pid, os.SCHED_FIFO, 
                             os.sched_param(priority))
        print(f"✅ Set realtime priority {priority} for PID {pid}")
    except PermissionError:
        print(f"⚠️  Need sudo for realtime priority. Run with: sudo python main.py")
    except Exception as e:
        print(f"⚠️  Could not set realtime priority: {e}")

# In main thread setup:
set_realtime_priority(os.getpid(), priority=50)
```

### Change 4: Telemetry as Display-Only
**Files**: `scripts/main.py` (telemetry_broadcast function)

Change the comment to clarify it's non-critical:
```python
def telemetry_broadcast():
    """
    NON-CRITICAL PATH (20Hz)
    Broadcasts telemetry for DISPLAY ONLY.
    Motor control has independent 100Hz loop.
    Not used for RPM feedback control.
    """
    # ... existing code ...
```

### Change 5: Decouple wheel_sync from stale RPM
**Files**: `scripts/core/wheel_sync.py`

Instead of waiting for `get_rpm()` callback, pass fresh RPM directly:
```python
# BEFORE (in motor.py):
self.wheel_sync.correct_target_rpms(speed_l, speed_r, ...)

# AFTER:
# Pass fresh Pico RPM directly instead of relying on callback
fresh_rpm = {
    'front_left': pico_pkt.rpm_left if pico_pkt else 0,
    'front_right': pico_pkt.rpm_right if pico_pkt else 0,
    'rear_left': pico_pkt.rpm_left if pico_pkt else 0,
    'rear_right': pico_pkt.rpm_right if pico_pkt else 0,
}
self.wheel_sync.correct_target_rpms(speed_l, speed_r, 
    rpm_override=fresh_rpm, ...)
```

## Implementation Priority

### PHASE 1 (Fixes Stale UI Display)
- ✅ Add timestamps to socket events (2 locations)
- ✅ Add freshness check in UI (1 location)
- **Effort**: 30 minutes | **Impact**: UI shows current data

### PHASE 2 (Fixes RPM Limit Enforcement)
- ✅ Create fast motor control loop (100Hz independent)
- ✅ Use SCHED_FIFO priority
- ✅ Pass fresh Pico data directly to motor control
- **Effort**: 2 hours | **Impact**: RPM limits respected

### PHASE 3 (Cleanup & Optimization)
- ✅ Mark telemetry_broadcast as non-critical
- ✅ Optimize wheel_sync RPM flow
- ✅ Add monitoring/logging for loop frequencies
- **Effort**: 1 hour | **Impact**: System clarity

## Validation Tests

After implementing, test with:

```bash
# Test 1: Check timestamp freshness
# Watch the console logs to verify timestamp_ms is within 20-50ms

# Test 2: Check RPM limits enforced
# Manually set gear to "1" (75 RPM max)
# Try to drive fast
# Verify car tops out at ~75 RPM, not higher

# Test 3: Check UI response time
# Watch the RPM gauge while driving
# Should be smooth and real-time (no jumping)

# Test 4: Monitor thread frequencies
# Add logging to each loop
# Verify: encoder_and_power = 50Hz, motor_control = 100Hz, telemetry = 20Hz
```

## Expected Outcomes

| Issue | Before | After |
|-------|--------|-------|
| UI RPM Age | 100+ seconds old | <100ms | 
| RPM Limit Enforcement | Ignored | Respected |
| Gauge Responsiveness | Stuttery / delayed | Smooth / immediate |
| Motor Control Stability | Varies | Consistent |

## Architecture Impact

```
OLD (Coupled):
Pico → encoder_thread → car_state → (blocks both motor & telemetry)
                           ↓
                    motor control waits for telemetry
                           
NEW (Decoupled):
Pico → motor_control_loop (100Hz) → Motor PWM (respects limits)
       ↓
    encoder_thread (50Hz) → car_state → telemetry (display only)
```

## Potential Issues & Mitigations

| Issue | Mitigation |
|-------|-----------|
| Realtime priority requires sudo | Document requirement, offer fallback |
| UART buffer still delayed | Add serial buffer monitoring |
| Network latency adds lag | Timestamps help detect/handle |
| React rendering still slow | Implement memoization in UI components |
| Car state inconsistencies | Use thread locks when accessing motor data |

## Summary

The bugs are caused by **tight coupling** between critical motor control and non-critical telemetry. The fixes involve:
1. **Decoupling** paths so motor control doesn't wait for telemetry
2. **Adding timestamps** so UI can detect stale data
3. **Using realtime scheduling** so control loop isn't preempted
4. **Passing fresh data** directly to motor control instead of via cache

Total implementation time: ~3-4 hours for full solution.
