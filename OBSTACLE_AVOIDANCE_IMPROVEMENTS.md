# Obstacle Avoidance Improvements

## Changes Made

### 1. **Faster Control Loop**
- **Before**: 50ms sleep (20 Hz update rate)
- **After**: 20ms sleep (50 Hz update rate)
- **Impact**: 2.5x faster reaction time = car responds to obstacles sooner

### 2. **Smooth Steering Ramp**
- **Before**: Instant jump to 85° steering angle
- **After**: Gradual ramp-up over 3-4 cycles (60-80ms)
  - Ramps up at 30°/cycle towards 85°
  - Ramps down at 20°/cycle when obstacle clears
- **Impact**: Smoother steering input prevents jolts and loss of traction

### 3. **Intelligent Deceleration During Avoidance**
- **Before**: Maintains full speed during sharp 85° swerve
- **After**: Automatically reduces speed proportional to steering angle
  - At 85° swerve: reduces to ~60% of gear speed
  - Linearly interpolates between full speed and 60%
- **Impact**: Better grip and control during sharp turns

### 4. **Obstacle Clearance Detection**
- **Before**: Avoidance angle persists until new obstacle detected
- **After**: Automatically releases when obstacles clear
  - Smoothly returns to user's manual steering angle
  - Prevents "stuck" avoidance mode
- **Impact**: More responsive to changing situations

### 5. **Debounced Obstacle Detection**
- **Before**: Could re-trigger avoidance every 50ms if oscillating in IR beam
- **After**: 5-cycle cooldown (100ms) between re-triggers
- **Impact**: Prevents rapid steering oscillations

### 6. **User Steering Preservation**
- **New**: Stores `user_steer_angle` separately from `steer_angle`
- **Impact**: When obstacle avoidance ends, returns to user's requested angle (not 0°)

---

## Code Changes Summary

### main.py
```python
# Added to car_state:
"user_steer_angle": 0           # User's manual steering input
"obstacle_avoidance_active": False  # Track avoidance state

# Updated physics_loop():
# - Smooth steering ramp-up/down
# - Obstacle cooldown mechanism
# - Speed reduction during avoidance (0.6x to 1.0x multiplier)
# - Faster loop: time.sleep(0.02) instead of 0.05

# Updated /steer/<angle> endpoint:
# - Now stores user input in "user_steer_angle"
# - Preserves this when obstacle avoidance is active
```

### motor.py
```python
# Added to __init__():
self.user_steering_angle = 0
self.obstacle_avoidance_active = False
self.obstacle_ramp_angle = 0
self.last_left_obstacle = False
self.last_right_obstacle = False
self.obstacle_cooldown = 0

# Updated check_obstacles():
# - Prints only on state changes (cleaner logs)

# Updated update():
# - Complete rewrite with smooth ramping
# - Automatic avoidance release logic
# - Speed reduction during swerve

# Updated set_steering():
# - Now stores in "user_steering_angle"
```

---

## Behavior Summary

### Before Detection
- Moving straight at full speed
- User manually steering (or not)

### When Obstacle Detected (e.g., left side)
1. **Cycle 1**: Steering begins ramping RIGHT (0° → 30°)
2. **Cycle 2**: Steering continues (30° → 60°), speed begins reducing
3. **Cycle 3**: Steering reaches target (60° → 85°), speed at 60%
4. **Cycles 4-N**: Maintains 85° and 60% speed while obstacle present

### When Obstacle Clears
1. Avoidance flag disables
2. Steering smoothly ramps back down: 85° → 65° → 45° → 25° → user_angle
3. Speed ramps back to gear's target
4. Car returns to user's last requested steering angle

### Example: Obstacle in middle of lane
- **t=0ms**: Obstacle detected, print "LEFT OBSTACLE - SWERVING RIGHT!"
- **t=20ms**: Steering at 30°, speed at 90% power
- **t=40ms**: Steering at 60°, speed at 75% power
- **t=60ms**: Steering at 85°, speed at 60% power (maintaining swerve)
- **t=200ms**: Obstacle clears, print "OBSTACLE CLEARED"
- **t=220ms**: Steering ramps to 65°, speed increasing
- **t=280ms**: Steering at 25°, speed at 100%
- **t=340ms**: Returns to user's angle (e.g., 0° straight)

---

## Configuration Tuning

If you need to adjust behavior, modify these values in `main.py`:

```python
AVOID_SWERVE_ANGLE = 85    # Max swerve angle (increase for more aggressive)
ACCEL_RATE = 2.0            # Speed increase rate
COAST_RATE = 0.5            # Speed decrease rate

# In physics_loop():
ramp_speed = 30             # Steering ramp speed (degrees per cycle)
obstacle_cooldown = 5       # Prevent re-trigger (in 20ms cycles)
speed_reduction = 0.6       # Minimum speed during 85° swerve (0.0-1.0)
```

---

## Testing Recommendations

1. **Test at different gears**:
   - Gear 1 (40 max) should easily handle 85° at reduced speed
   - Gear 3 (80 max) → 48 actual speed during swerve

2. **Test with obstacles in different positions**:
   - Center of lane
   - Far left/right edges
   - Multiple obstacles

3. **Verify smooth operation**:
   - Look for no jerky steering transitions
   - Check that car maintains forward progress while swerving
   - Confirm it doesn't overshoot and hit obstacle

4. **Monitor logs**:
   - Should see "LEFT OBSTACLE" then "OBSTACLE CLEARED"
   - Should NOT see rapid re-triggers

