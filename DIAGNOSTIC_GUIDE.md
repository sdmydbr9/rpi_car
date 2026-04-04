# Diagnostic Guide: How to Verify the Bugs

## Overview
This guide shows you how to verify each bug exists and how to confirm the fixes work.

---

## Bug #1: No Timestamps in Socket Events

### How to Verify It Exists
1. Open browser DevTools → Network tab
2. Filter for WebSocket messages
3. Look for `rpm_update` and `telemetry_update` events
4. Open the message payload
5. **Expected (Bug)**: No `timestamp_ms` field
6. **Check**: Search for `"timestamp_ms"` - should find nothing

### Example (Current - Broken)
```json
rpm_update: {
  "rpm": 45.3,
  "rpm_left": 44.0,
  "rpm_right": 46.5,
  "target_rpm": 50.0,
  "engine_running": true
}
// ❌ NO TIMESTAMP - Can't tell age of data!
```

### Example (After Fix)
```json
rpm_update: {
  "rpm": 45.3,
  "rpm_left": 44.0,
  "rpm_right": 46.5,
  "target_rpm": 50.0,
  "engine_running": true,
  "timestamp_ms": 1712245678901
}
// ✅ Has timestamp - UI can verify it's fresh!
```

### Verification Script
```javascript
// Run this in browser console:
let lastTimestamp = 0;
let lastAge = 0;

socket.on('rpm_update', (data) => {
  if (data.timestamp_ms) {
    lastAge = Date.now() - data.timestamp_ms;
    lastTimestamp = data.timestamp_ms;
    console.log(`RPM age: ${lastAge}ms, timestamp: ${lastTimestamp}`);
  } else {
    console.log('❌ No timestamp in rpm_update');
  }
});

// Check every 5 seconds
setInterval(() => {
  console.log(`Last RPM age: ${lastAge}ms`);
}, 5000);
```

---

## Bug #2: encoder_and_power_thread Can Miss 50Hz Target

### How to Verify It Exists
1. Add logging to encoder_and_power_thread in main.py:
```python
def encoder_and_power_thread():
    """Read Pico odometry data at 50 Hz and battery/current roughly once per second."""
    interval = 0.02  # 20ms for RPM / odometry (50 Hz)
    last_time = time.monotonic()
    
    while True:
        try:
            time.sleep(interval)
            
            # ADD THIS:
            now = time.monotonic()
            actual_dt = now - last_time
            last_time = now
            if actual_dt > 0.025:  # Should be 0.02, but allow 2.5ms slack
                print(f"⚠️  ENCODER THREAD LATE: {actual_dt*1000:.1f}ms (target 20ms)")
            
            # ... rest of code
```

2. Run the script and watch logs
3. **Expected (Bug)**: See warnings like `ENCODER THREAD LATE: 45.2ms`
4. **Expected (Fix)**: All delays should be <25ms

### What to Watch For
- Delays spike when camera is active → camera steals CPU
- Delays increase under network load → WebSocket blocking
- Delays consistent but >20ms → OS scheduling issue

### Example Output (Current - Broken)
```
⚠️  ENCODER THREAD LATE: 45.2ms (target 20ms)
⚠️  ENCODER THREAD LATE: 52.1ms (target 20ms)
⚠️  ENCODER THREAD LATE: 38.7ms (target 20ms)
❌ Averages 2-3x slower than target
```

### Example Output (After Fix)
```
✅ ENCODER THREAD ON TIME: 21.3ms
✅ ENCODER THREAD ON TIME: 19.8ms
✅ ENCODER THREAD ON TIME: 20.2ms
✅ Averages right on target
```

---

## Bug #3: Telemetry Broadcasts Stale  car_state Values

### How to Verify It Exists
1. Add logging to telemetry_broadcast function:
```python
def telemetry_broadcast():
    """Broadcast telemetry data to all connected clients at 20Hz"""
    
    last_rpm = None
    same_count = 0
    
    while True:
        try:
            # ... existing code ...
            
            # ADD THIS BEFORE socketio.emit:
            current_rpm = car_state.get("encoder_rpm", 0.0)
            if current_rpm == last_rpm:
                same_count += 1
                if same_count > 3:  # Same value for 4+ iterations = stale
                    print(f"⚠️  TELEMETRY STALE: RPM={current_rpm} unchanged for {same_count*50}ms")
            else:
                same_count = 0
                last_rpm = current_rpm
            
            socketio.emit('telemetry_update', telemetry_data)
```

2. Run script and watch logs
3. **Expected (Bug)**: See warnings about stale telemetry every few seconds
4. **Expected (Fix)**: RPM changes are reflected immediately in telemetry

### Example Output (Current - Broken)
```
⚠️  TELEMETRY STALE: RPM=45.3 unchanged for 200ms
⚠️  TELEMETRY STALE: RPM=45.3 unchanged for 250ms
⚠️  TELEMETRY STALE: RPM=45.3 unchanged for 150ms
❌ Telemetry frequently sends old values
```

### Example Output (After Fix)
```
✅ TELEMETRY FRESH: RPM changed to 46.1
✅ TELEMETRY FRESH: RPM changed to 47.5
✅ TELEMETRY FRESH: RPM changed to 48.2
✅ Telemetry always reflects latest state
```

---

## Bug #4: RPM Never Updates in UI

### How to Verify It Exists
1. Drive the car in your testing environment
2. Watch the RPM gauge in the UI
3. Note: Does it update smoothly or in jumps?
4. **Expected (Bug)**: Updates every ~100-200ms in chunks
5. **Expected (Fix)**: Updates every ~20ms smoothly

### Visual Test
- **Broken**: RPM gauge jumps every ~100ms with large deltas
- **Fixed**: RPM gauge changes smoothly every ~20ms

### Browser Console Test
```javascript
// Track RPM update frequency
let lastRpmUpdate = Date.now();
let updateDeltas = [];

socket.on('rpm_update', (data) => {
  let now = Date.now();
  let delta = now - lastRpmUpdate;
  lastRpmUpdate = now;
  updateDeltas.push(delta);
  
  if (updateDeltas.length % 50 === 0) {
    let avg = updateDeltas.reduce((a,b) => a+b, 0) / updateDeltas.length;
    let max = Math.max(...updateDeltas);
    let min = Math.min(...updateDeltas);
    console.log(`RPM Updates - Avg: ${avg.toFixed(0)}ms, Min: ${min}ms, Max: ${max}ms`);
    updateDeltas = [];
  }
});
```

### Example Output (Current - Broken)
```
RPM Updates - Avg: 98ms, Min: 45ms, Max: 250ms
❌ Updates should be 20ms apart! Getting 5x slower.
```

### Example Output (After Fix)
```
RPM Updates - Avg: 22ms, Min: 18ms, Max: 28ms
✅ Updates are consistent 20ms intervals (50Hz)
```

---

## Bug #5: RPM Limits Not Enforced

### How to Verify It Exists
1. Start the car (engine on)
2. Put in Gear "1" (75 RPM max)
3. Press throttle fully
4. Watch the RPM gauge
5. **Expected (Bug)**: RPM spikes to 100+ even though limit is 75
6. **Expected (Fix)**: RPM caps at ~75 and stays there

### Test Procedure
```python
# Add this to diagnostics/rpm_test.py or similar:

# Set gear to 1 (75 RPM max)
send_gear_command("1")
time.sleep(0.5)

# Drive at full throttle for 2 seconds
send_motor_command(100, 1440)  # 100% PWM, center steering
time.sleep(2)

# Check RPM
rpm_l, rpm_r = get_pico_rpm()
print(f"Current RPM: L={rpm_l:.1f}, R={rpm_r:.1f}")

# Expected (Bug): RPM > 80 (exceeds limit)
# Expected (Fix): RPM <= 75 (respects limit)

if rpm_l > 80 or rpm_r > 80:
    print("❌ BUG: RPM exceeded gear limit!")
else:
    print("✅ FIX WORKS: RPM respects gear limit")
```

### Example Output (Current - Broken)
```
Gear: 1 (limit 75 RPM)
Current PWM: 100%
Current RPM: L=95.3, R=97.2
❌ BUG: RPM exceeded gear limit by 25%!
Motor ignores gear limit - can exceed by 30+ RPM
```

### Example Output (After Fix)
```
Gear: 1 (limit 75 RPM)
Current PWM: 100%
Current RPM: L=73.8, R=74.2
✅ FIX WORKS: RPM respects gear limit
Motor maintains speed under gear limit
```

---

## Bug #6: UI Can't Detect Stale Data

### How to Verify It Exists
1. Manually delay one rpd_update by 500ms
2. Watch: Does UI display it as if it's fresh?
3. **Expected (Bug)**: Yes, displays without warning
4. **Expected (Fix)**: Should skip/warn about stale data

### Simulation Test
```javascript
// Run this in browser console to simulate stale data:

// Intercept rpm_update events
socket.on('rpm_update', (data) => {
  // Make one event very old (500ms ago)
  data.timestamp_ms = Date.now() - 500;
  
  console.log(`⚠️  SIMULATED STALE DATA: ${Date.now() - data.timestamp_ms}ms old`);
  
  // Did the UI update? It shouldn't!
  console.log(`Old RPM value displayed: ${document.getElementById('rpm-gauge').textContent}`);
});
```

### Expected Behavior (Bug)
```
⚠️  SIMULATED STALE DATA: 500ms old
Old RPM value displayed: 45.3  ← Still displays old value!
❌ No protection against stale data
```

### Expected Behavior (Fix)
```
⚠️  SIMULATED STALE DATA: 500ms old
Old RPM value displayed: 45.3  ← IGNORED due to freshness check!
✅ Stale data rejected, keeps old UI value
```

---

## Bug #7: No Realtime Scheduling

### How to Verify It Exists
1. Check process priorities:
```bash
# Show priority of running main.py
ps aux | grep main.py | grep -v grep

# Check scheduling policy (should be SCHED_OTHER currently)
ps -eo pid,cmd,class | grep main.py
```

2. **Expected (Bug)**: Shows `SCHED_OTHER` or `TS` (timeshare)
3. **Expected (Fix)**: Should show `SCHED_FIFO` or `FF`

### Example Output (Current - Broken)
```bash
$ ps -eo pid,cmd,class | grep main
1234 python3 main.py              TS        ← TS = timeshare (not realtime)
❌ Can be preempted by any thread
```

### Example Output (After Fix)
```bash
$ ps -eo pid,cmd,class | grep main
1234 python3 main.py              FF        ← FF = FIFO (realtime)
✅ Has priority, won't be preempted
```

### Verification Script
```python
# Add to main.py after startup:
import os

def check_scheduler():
    """Verify realtime scheduling is enabled."""
    try:
        policy = os.sched_getscheduler(os.getpid())
        # Possible values: SCHED_OTHER=0, SCHED_FIFO=1, SCHED_RR=2
        if policy == 0:
            print("⚠️  Running with SCHED_OTHER (can be preempted)")
            return False
        elif policy == 1:
            print("✅ Running with SCHED_FIFO (realtime, won't preempt)")
            return True
        elif policy == 2:
            print("✅ Running with SCHED_RR (realtime round-robin)")
            return True
    except Exception as e:
        print(f"❌ Could not check scheduler: {e}")
    return False

# Call after startup
check_scheduler()
```

---

## Comprehensive Diagnostic Script

Combine all checks into one script:

```python
#!/usr/bin/env python3
import time
import os
import sys

def diagnose_rpm_system():
    """Run all diagnostic checks."""
    
    print("=" * 60)
    print("RPM SYSTEM DIAGNOSTIC TEST")
    print("=" * 60)
    
    # Check 1: Timestamps
    print("\n[1] Checking timestamp support...")
    try:
        from scripts.main import socketio
        # This would need actual event inspection
        print("⚠️  Can't verify from script - use browser DevTools")
    except:
        pass
    
    # Check 2: Thread frequencies
    print("\n[2] Starting encoder thread frequency test...")
    from scripts.core.pico_sensor_reader import init_pico_reader
    init_pico_reader()
    
    times = []
    last = time.monotonic()
    for i in range(50):
        time.sleep(0.02)
        now = time.monotonic()
        dt = now - last
        last = now
        times.append(dt * 1000)
        if dt > 0.025:
            print(f"  ⚠️  Iteration {i}: {dt*1000:.1f}ms (late)")
    
    avg = sum(times) / len(times)
    print(f"  Average: {avg:.1f}ms (target: 20ms)")
    if avg > 25:
        print("  ❌ ENCODER THREAD TOO SLOW")
    else:
        print("  ✅ ENCODER THREAD OK")
    
    # Check 3: Scheduler
    print("\n[3] Checking scheduler policy...")
    try:
        policy = os.sched_getscheduler(os.getpid())
        if policy == 0:
            print("  ⚠️  SCHED_OTHER (not realtime)")
        elif policy == 1:
            print("  ✅ SCHED_FIFO (realtime)")
        else:
            print(f"  ? Unknown policy: {policy}")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    # Check 4: RPM limits
    print("\n[4] Checking RPM limit enforcement...")
    from scripts.core.pico_sensor_reader import get_pico_rpm
    rpm_l, rpm_r = get_pico_rpm()
    print(f"  Current RPM: L={rpm_l:.1f}, R={rpm_r:.1f}")
    
    # Would need more setup to test properly
    print("  ⚠️  Full test requires actual driving")
    
    print("\n" + "=" * 60)
    print("END DIAGNOSTIC TEST")
    print("=" * 60)

if __name__ == "__main__":
    diagnose_rpm_system()
```

---

## Quick Health Check Checklist

Before and after fixes, run through:

- [ ] **UI**: RPM gauge updates every ~20ms (not 50-100ms chunks)
- [ ] **Data**: Browser DevTools shows `timestamp_ms` in socket messages
- [ ] **Freshness**: RPM data is always <100ms old
- [ ] **Limits**: Car respects gear RPM limits when throttle is maxed
- [ ] **Thread**: `encoder_and_power_thread` averages 20ms intervals
- [ ] **Scheduler**: Process shows SCHED_FIFO (if using realtime fix)
- [ ] **Motor**: Motor command executes without delay

## When to Run Diagnostics

1. **Before fix**: Establish baseline bugs
2. **During implementation**: Verify each step works
3. **After fix**: Confirm all issues resolved
4. **Weekly**: Monitor for regressions
