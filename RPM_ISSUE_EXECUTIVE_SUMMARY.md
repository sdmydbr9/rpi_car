# Executive Summary: RPM Display & Motor Control Bugs

## The Core Problem

**Your car's UI shows RPM data that's 100+ seconds old, and the RPM limits (gear-based speed caps) are being ignored.**

This isn't a performance issue—it's an **architectural design flaw** where the motor control system and UI telemetry system are tightly coupled, causing delays to cascade from display to motors.

## What's Happening

```
Pico Sensor (instant) 
    ↓
Pico Reader (50Hz, real-time)
    ↓
encoder_and_power_thread (50Hz target, but can get delayed)
    ↓
car_state dictionary (acts as cache)
    ↓
telemetry_broadcast (20Hz, sends to UI)
    ↓
Network / UI rendering (adds more delay)
    ↓
UI Display (100+ seconds old!)
    ↓
ALSO used by motor_control!
    ↓
Motor ignores RPM limits because it thinks RPM is still high
```

## Why RPM Limits Are Ignored

The motor control system (wheel_sync) checks the current RPM to enforce gear limits. But:
1. It reads RPM from `car_state` (which may be stale)
2. If `car_state` hasn't been updated recently (due to telemetry delays), it gets outdated values
3. The PID controller thinks the car is still moving fast, so it doesn't apply brakes
4. Car exceeds gear limits

## The 7 Hidden Bugs

| # | Bug | Location | Impact |
|---|-----|----------|---------|
| 1 | No timestamps in data packets | main.py, socketClient.ts | UI can't detect stale data |
| 2 | encoder thread can be preempted | main.py line 6178 | RPM updates aren't guaranteed 50Hz |
| 3 | Telemetry reads cached car_state | main.py line 6370 | Telemetry amplifies delays |
| 4 | Pico packet buffer delays | pico_sensor_reader.py | Packets may be 100ms+ old |
| 5 | Motor control depends on car_state | wheel_sync.py | Stale telemetry breaks motor control |
| 6 | No freshness check in UI | CockpitController.tsx | Old data displayed without warning |
| 7 | No realtime priorities | main.py startup | Threads can be starved by OS |

## Why We Detected This

When you checked:
- Pico sensor reader: **INSTANT** (correct ✓)
- UI: **100+ seconds old** (wrong ✗)

This huge gap proves data is being buffered/delayed between sensor and display. And since motor control uses the same delayed data path, it also gets stale RPM feedback.

## The Fix (High-Level)

### Step 1: Add Data Freshness Tracking
Add millisecond timestamps to every data packet so the UI can detect when data is stale.

### Step 2: Decouple Motor Control from Telemetry
Create a **dedicated, high-priority motor control loop** that:
- Reads fresh Pico data directly (not through car_state)
- Updates motor PWM at 100Hz (vs telemetry at 20Hz)
- Uses `SCHED_FIFO` realtime priority (never gets preempted)
- Stays independent of UI/network delays

### Step 3: Make Telemetry Display-Only
Keep the telemetry broadcast for UI display, but **don't use it for motor control**. It becomes non-critical.

```
BEFORE (Broken):
Pico → motor_control → telemetry → UI
       (all coupled, delays cascade)

AFTER (Fixed):
Pico → motor_control (100Hz, SCHED_FIFO) → Motor PWM ✓
  ↓
  telemetry (20Hz) → UI (with timestamps) ✓
```

## Expected Outcomes

| Metric | Before | After |
|--------|--------|-------|
| UI RPM display latency | 100+ seconds | <100ms |
| Motor control RPM latency | 100+ seconds | <10ms |
| Gear limit enforcement | ❌ Ignored | ✅ Respected |
| UI responsiveness | Stuttery | Smooth |
| Car behavior | Unpredictable | Predictable |

## Implementation Effort

| Phase | Changes | Time | Priority |
|-------|---------|------|----------|
| Phase 1: Add timestamps | 2 files, 3 locations | 30 min | HIGH |
| Phase 2: Decouple motor loop | 2 files, 1 new function | 2 hours | HIGH |
| Phase 3: Cleanup & monitoring | 2 files, logging | 1 hour | MEDIUM |

**Total: 3.5 hours for complete fix**

## Files That Need Changes

```
scripts/main.py
├─ Add timestamps to rpm_broadcast() [~5 lines]
├─ Add timestamps to telemetry_broadcast() [~1 line]
└─ Create fast_motor_control_loop() [~30 lines]

scripts/core/wheel_sync.py
└─ Accept fresh_rpm parameter [~5 lines]

src/lib/socketClient.ts
└─ Add timestamp_ms to RpmUpdateData interface [~1 line]

src/components/cockpit/CockpitController.tsx
└─ Add freshness check to onRpmUpdate() [~10 lines]
```

## The Architectural Root Cause

The system was designed with a **single shared state** model:

```python
car_state = {  # This dictionary is used for EVERYTHING
    "encoder_rpm": 45.0,         # ... used by motor control
    "encoder_rpm_left": 44.0,    # ... also used by telemetry
    "encoder_rpm_right": 46.0,   # ... also used by display
}
```

This works fine when everything is fast, but when telemetry gets delayed (network, rendering, OS scheduling), **the motor control starves** because it depends on the same cache.

The fix is to **split into two paths**:
- **Critical path**: Pico → Motor Control (independent, realtime)
- **Display path**: car_state → Telemetry → UI (can be slower)

## Why This Wasn't Obvious Before

1. **Immediate data looks correct**: When you run the Pico sensor script standalone, data IS real-time (Pico→Reader is working)
2. **Cascade effect hidden**: The delay doesn't show up until data flows through multiple layers
3. **Coupled architecture**: Motor control and telemetry share the same code path, so delays in one affect both
4. **Network queueing**: WebSocket events can batch/queue, hiding where delays originate

## Next Steps

1. **Read**: Full technical analysis in `/home/pi/rpi_car/PERFORMANCE_ANALYSIS_RPM_DISPLAY.md`
2. **Plan**: Review implementation blueprint in `/home/pi/rpi_car/RPM_DISPLAY_IMPLEMENTATION_BLUEPRINT.md`
3. **Implement**: Start with Phase 1 (timestamps) for immediate UI fix
4. **Test**: Run validation tests to confirm RPM limits are now respected
5. **Monitor**: Add logging to verify thread frequencies post-implementation

## Bottom Line

**You have an architectural bug, not a performance bug.** The solution is to decouple the motor control system (critical) from the telemetry system (non-critical), add data freshness tracking, and use realtime scheduling for the control loop.

This will give you:
- ✅ Fresh RPM data in UI (<100ms old)
- ✅ Respected RPM limits (motor control always gets fresh feedback)
- ✅ Smooth, predictable car behavior
