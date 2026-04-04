# Quick Reference: Code Changes Needed

## File-by-File Change Checklist

### 1. `scripts/main.py`

#### Location A: rpm_broadcast() function (~line 6351)
```python
# FIND THIS:
socketio.emit('rpm_update', {
    "rpm": rpm_avg,
    "rpm_left": rpm_left,
    "rpm_right": rpm_right,
    "target_rpm": round(float(car_state.get("current_target_rpm", 0.0) or 0.0), 1),
    "engine_running": car_state["engine_running"],
})

# ADD THIS BEFORE socketio.emit():
import time
socketio.emit('rpm_update', {
    "rpm": rpm_avg,
    "rpm_left": rpm_left,
    "rpm_right": rpm_right,
    "target_rpm": round(float(car_state.get("current_target_rpm", 0.0) or 0.0), 1),
    "engine_running": car_state["engine_running"],
    "timestamp_ms": int(time.time() * 1000),  # ADD THIS LINE
})
```
**Lines affected**: ~6351 (add 1 line)

#### Location B: telemetry_broadcast() function (~line 6410)
```python
# FIND THIS:
socketio.emit('telemetry_update', telemetry_data)

# CHANGE TO THIS:
telemetry_data["timestamp_ms"] = int(time.time() * 1000)
socketio.emit('telemetry_update', telemetry_data)
```
**Lines affected**: ~6410 (add 1 line before emit)

#### Location C: Start of main script section (~line 6500)
```python
# FIND THIS SECTION WHERE THREADS ARE STARTED:
rpm_thread = threading.Thread(target=rpm_broadcast, daemon=True)
rpm_thread.start()
print("🔄 [Telemetry] ✅ RPM broadcast thread started (20ms interval)")

# ADD THIS NEW THREAD DEFINITION AFTER:
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
            
            # Read FRESH Pico packet directly
            pico_pkt = pico_get_sensor_packet()
            if pico_pkt is None:
                continue
            
            # Get current target RPM from car_state
            target_rpm = car_state.get("current_target_rpm", 0.0)
            
            # Call motor PID with fresh RPM data
            car_system.rpm_pid_tick(target_rpm)
            
            # Apply motor commands
            pwm_l, pwm_r = car_system.get_pwm()
            send_lr_pwm(int(pwm_l), int(pwm_r))
            
        except Exception as e:
            print(f"❌ Fast motor control error: {e}")
            time.sleep(interval)

# Start motor control thread AFTER rpm_thread:
motor_thread = threading.Thread(target=fast_motor_control_loop, daemon=True)
motor_thread.start()
print("🚀 [Control] Fast motor control loop started (100Hz, SCHED_FIFO)")

# Optional: Set realtime priority (requires sudo)
try:
    import os
    os.sched_setscheduler(os.getpid(), os.SCHED_FIFO, os.sched_param(50))
    print("✅ Motor thread set to SCHED_FIFO priority")
except PermissionError:
    print("⚠️  Need sudo for realtime priority. Run with: sudo python main.py")
except Exception as e:
    print(f"⚠️  Could not set realtime priority: {e}")
```
**Lines affected**: ~30 lines added

---

### 2. `src/lib/socketClient.ts`

#### Location: RpmUpdateData interface (~line 132)
```typescript
// FIND THIS:
export interface RpmUpdateData {
  rpm: number;
  rpm_left?: number;
  rpm_right?: number;
  target_rpm?: number;
  engine_running?: boolean;
}

// CHANGE TO THIS:
export interface RpmUpdateData {
  rpm: number;
  rpm_left?: number;
  rpm_right?: number;
  target_rpm?: number;
  engine_running?: boolean;
  timestamp_ms?: number;  // ADD THIS LINE
}
```
**Lines affected**: ~1 line added

---

### 3. `src/components/cockpit/CockpitController.tsx`

#### Location: socketClient.onRpmUpdate() callback (~line 799)
```typescript
// FIND THIS:
socketClient.onRpmUpdate((data) => {
  const engineRunning = data.engine_running ?? isEngineRunning;
  setControlState(prev => ({
    ...prev,
    rpm: engineRunning ? (data.rpm ?? prev.rpm) : 0,
    rpmLeft: engineRunning ? (data.rpm_left ?? prev.rpmLeft) : 0,
    rpmRight: engineRunning ? (data.rpm_right ?? prev.rpmRight) : 0,
    targetRpm: engineRunning ? (data.target_rpm ?? prev.targetRpm) : 0,
  }));
});

// CHANGE TO THIS:
socketClient.onRpmUpdate((data) => {
  // Check if data is fresh (add this block)
  const now = Date.now();
  const ageMs = data.timestamp_ms ? now - data.timestamp_ms : 0;
  const isStale = ageMs > 200; // 200ms = stale threshold
  
  // Only update if data is fresh
  if (!isStale) {
    const engineRunning = data.engine_running ?? isEngineRunning;
    setControlState(prev => ({
      ...prev,
      rpm: engineRunning ? (data.rpm ?? prev.rpm) : 0,
      rpmLeft: engineRunning ? (data.rpm_left ?? prev.rpmLeft) : 0,
      rpmRight: engineRunning ? (data.rpm_right ?? prev.rpmRight) : 0,
      targetRpm: engineRunning ? (data.target_rpm ?? prev.targetRpm) : 0,
    }));
  }
});
```
**Lines affected**: ~10 lines added/modified

---

### 4. `scripts/core/wheel_sync.py` (Optional - Phase 3)

#### Location: _correct_impl() method (~line 358)
```python
# FIND THIS LINE:
rpm = self.get_rpm()

# CONSIDER ADDING THIS PARAMETER:
def _correct_impl(
    self,
    speed_l: float,
    speed_r: float,
    target_rpm_l: float,
    target_rpm_r: float,
    duty_cap: float = 63.0,
    fresh_rpm: dict = None,  # ADD THIS PARAMETER
):
    """Shared correction path for PWM-derived and explicit RPM targets."""
    # USE FRESH RPM IF PROVIDED:
    if fresh_rpm is not None:
        rpm = fresh_rpm
    else:
        rpm = self.get_rpm()
```
**Lines affected**: ~5 lines modified (optional for Phase 2)

---

## Summary of Changes by Priority

### PHASE 1 (UI Fix - 30 minutes)
- [ ] scripts/main.py: rpm_broadcast() - add timestamp (line 6351)
- [ ] scripts/main.py: telemetry_broadcast() - add timestamp (line 6410)
- [ ] src/lib/socketClient.ts: Add timestamp_ms to RpmUpdateData interface
- [ ] src/components/cockpit/CockpitController.tsx: Add freshness check to onRpmUpdate()

### PHASE 2 (Motor Control Fix - 2 hours)
- [ ] scripts/main.py: Create fast_motor_control_loop() function (~line 6500)
- [ ] scripts/main.py: Start motor control thread with proper logging
- [ ] scripts/main.py: Set SCHED_FIFO priority (optional but recommended)
- [ ] scripts/core/wheel_sync.py: Modify to accept fresh_rpm parameter (optional)

### PHASE 3 (Cleanup - 1 hour)
- [ ] Add monitoring/logging for thread frequencies
- [ ] Document architecture changes in code comments
- [ ] Test and validate

---

## Testing Commands

After making changes, verify:

```bash
# Test 1: Check timestamps in network traffic
# Use browser DevTools → Network tab → WebSocket messages
# Should see "timestamp_ms": <number> in each message

# Test 2: Check thread frequencies
# Add logging like: print(f"[{time.time()}] RPM: {rpm_l}, {rpm_r}")
# Verify: encoder_and_power = 50Hz, motor_control = 100Hz, telemetry = 20Hz

# Test 3: Check RPM limits enforced
# Set gear to "1" (75 RPM max)
# Drive forward at full throttle
# Verify car tops out at ~75 RPM

# Test 4: Check UI responsiveness
# Watch RPM gauge while driving fast
# Should update smoothly every ~20ms (not every ~100ms)
```

---

## Validation Checklist

- [ ] Phase 1: UI shows RPM data within <100ms age (check DevTools)
- [ ] Phase 2: Motor control loop runs at 100Hz (check logs)
- [ ] Phase 2: RPM limits respected in each gear
- [ ] Phase 3: No new errors in console or logs
- [ ] Full test drive: Smooth, predictable acceleration/braking

---

## Rollback Plan

If issues arise:
1. Remove the timestamp checks from React (falls back to old behavior)
2. Comment out fast_motor_control_loop (keeps original encoder thread)
3. The system will return to previous state

---

## Common Pitfalls to Avoid

1. **Don't forget thread imports**: Fast motor loop needs `pico_get_sensor_packet`, `send_lr_pwm`
2. **Don't set SCHED_FIFO without sudo**: Will fail silently, still runs at normal priority
3. **Don't remove encoder_and_power_thread**: Still needed for odometry, sensors, telemetry
4. **Don't hardcode timestamps**: Use `int(time.time() * 1000)` consistently
5. **Don't skip the freshness check**: Without it, old data can still break motor control

---

## Expected File Sizes After Changes

| File | Before | After | Delta |
|------|--------|-------|-------|
| main.py | ~6800 lines | ~6835 lines | +35 lines |
| socketClient.ts | ~500 lines | ~502 lines | +2 lines |
| CockpitController.tsx | ~850 lines | ~860 lines | +10 lines |
| wheel_sync.py | ~600 lines | ~605 lines | ~+5 lines (opt) |

**Total additions**: ~50 lines across 4 files

---

## Questions Before Implementation?

Review:
1. `RPM_ISSUE_EXECUTIVE_SUMMARY.md` - Understand the problem
2. `PERFORMANCE_ANALYSIS_RPM_DISPLAY.md` - Technical details
3. `RPM_DISPLAY_IMPLEMENTATION_BLUEPRINT.md` - Architectural context

Then implement in order: Phase 1 → Phase 2 → Phase 3
