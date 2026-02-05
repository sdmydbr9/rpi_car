# Gear Control System Documentation

## Gear Specifications

| Gear | Speed Range | Default Speed | Description |
|------|-------------|---|---|
| **S** | 80-100% | 90% | Sport/Super speed - maximum acceleration |
| **3** | 50-80% | 65% | High gear - quick acceleration |
| **2** | 30-50% | 40% | Medium gear - balanced control |
| **1** | 0-30% | 15% | Low gear - careful/precise control |
| **N** | 0% | 0% | Neutral - no movement (safety) |
| **R** | Reverse | 15% | Reverse - backward movement |

## Control Behavior by Gear

### Forward Gears (S, 3, 2, 1)
- **Throttle Pedal** (Right): Move forward at gear's default speed
- **Brake Pedal** (Left): Stop the car
- **Steering**: Works normally during forward movement

### Reverse Gear (R)
- **Throttle Pedal** (Right): Move backward at 15% speed (reverse driving)
- **Brake Pedal** (Left): Gentle forward to ease out of reverse
- **Steering**: Works in reverse direction

### Neutral (N)
- **Throttle Pedal**: No effect - car won't move
- **Brake Pedal**: No effect - car already stopped
- **Steering**: No movement, just direction hold

## API Endpoints

### Gear Control
```
GET /gear/S    → Switch to Sport (80-100%)
GET /gear/3    → Switch to Gear 3 (50-80%)
GET /gear/2    → Switch to Gear 2 (30-50%)
GET /gear/1    → Switch to Gear 1 (0-30%)
GET /gear/N    → Switch to Neutral (stops car)
GET /gear/R    → Switch to Reverse (backward)
```

### Movement Control (with gear consideration)
```
GET /forward   → Accelerate forward (or backward if in Reverse)
GET /backward  → Brake (stop) or ease out of reverse
GET /left      → Turn left (works in all gears)
GET /right     → Turn right (works in all gears)
GET /stop      → Full stop
```

### Speed Control (direct PWM override, 0-100)
```
GET /speed/50  → Set motors to 50% power (bypasses gear limit)
```

## Frontend Usage

### Gear Shifter Component
The GearShifter component in the UI shows 6 positions:
- `R` - Reverse
- `N` - Neutral
- `1` - Low gear
- `2` - Medium gear
- `3` - High gear
- `S` - Sport

### Pedals
- **Right Pedal (Throttle)**: Accelerate forward or backward depending on gear
- **Left Pedal (Brake)**: Stop or ease out of reverse

### Steering Wheel
Works in all gears for directional control

## Logic Flow

```
1. Driver selects gear (S, 3, 2, 1, N, R)
   └─→ Backend sets current_gear and stops car

2. Driver presses throttle
   └─→ Check if gear == N: if yes, no movement
   └─→ Get speed for current gear
   └─→ Set PWM to gear's speed
   └─→ If reverse: move backward, else move forward

3. Driver turns steering wheel
   └─→ Send left/right commands (works in all gears)
   └─→ Car turns while moving

4. Driver presses brake
   └─→ If in reverse: gentle forward
   └─→ Else: full stop

5. Driver releases throttle
   └─→ Send stop command
   └─→ Car stops
```

## Safety Features

✅ **Neutral Lock** - Cannot move in Neutral gear, only steering  
✅ **Gear Shift Safety** - Car stops when shifting gears  
✅ **Speed Limiting** - Each gear limits maximum speed  
✅ **Reverse Control** - Gentle speeds to prevent damage  
✅ **Brake Override** - Brake always stops regardless of gear  

## Example Driving Sequences

### Forward Acceleration
```
1. Shift to Gear 1 → Car stopped, ready
2. Press Throttle → Move forward at 15%
3. Shift to Gear 2 → Car stops briefly
4. Press Throttle → Move forward at 40%
5. Shift to Gear 3 → Car stops briefly
6. Press Throttle → Move forward at 65%
```

### Reverse Parking
```
1. Shift to Gear R → Car stopped
2. Press Throttle → Move backward at 15%
3. Press Brake → Gentle forward to ease out
4. Shift to Gear N → Car stopped, fully parked
```

### Emergency Stop
```
1. Press Brake anytime → Car stops immediately
   (Works regardless of gear selection)
```

## Testing

Test all gears with:
```bash
# Forward gear testing
curl http://localhost:5000/gear/1
curl http://localhost:5000/forward  # Should move at 15%

# Reverse testing
curl http://localhost:5000/gear/R
curl http://localhost:5000/forward  # Should move backward at 15%

# Neutral testing
curl http://localhost:5000/gear/N
curl http://localhost:5000/forward  # Should return NEUTRAL_NO_MOVE

# Sport testing
curl http://localhost:5000/gear/S
curl http://localhost:5000/forward  # Should move at 90% (fast!)
```

## Speed Values (PWM %)

Current implementation uses these PWM percentages:
- **Gear S**: 90% → Very fast
- **Gear 3**: 65% → Fast
- **Gear 2**: 40% → Medium
- **Gear 1**: 15% → Slow
- **Gear R**: 15% → Slow (reverse)
- **Gear N**: 0% → Stopped

Adjust these in `main.py` under `GEAR_SPEEDS` dict if needed.
