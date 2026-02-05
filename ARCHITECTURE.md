# System Architecture & Communication Flow

## Overall Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Your Network                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────┐              ┌─────────────────────┐ │
│  │  Web Browser     │              │   Raspberry Pi      │ │
│  │  (Phone/PC)      │◄────HTTP────►│   (Car Control)     │ │
│  │                  │   Port 5000   │                     │ │
│  │  React Dashboard │              │  Flask Server       │ │
│  │  - Steering      │              │  - API Endpoints    │ │
│  │  - Gears         │              │  - Motor Control    │ │
│  │  - Throttle      │              │  - GPIO Interface   │ │
│  │  - Brake         │              │                     │ │
│  └──────────────────┘              └─────────────────────┘ │
│                                            │                │
│                                            ▼                │
│                                    ┌─────────────────┐      │
│                                    │  GPIO Pins      │      │
│                                    │  (Motor Driver) │      │
│                                    │  L298N Module   │      │
│                                    └─────────────────┘      │
│                                            │                │
│                                            ▼                │
│                                    ┌─────────────────┐      │
│                                    │  DC Motors      │      │
│                                    │  & Steering     │      │
│                                    │  Servo          │      │
│                                    └─────────────────┘      │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Communication Flow (Step by Step)

### 1. User Interaction
```
User turns steering wheel in browser dashboard
         ↓
React detects angle change event
         ↓
handleAngleChange() called with angle value
         ↓
sendCommand("steering", 45) called
```

### 2. API Request
```
sendCommand() builds HTTP request:
  - URL: http://192.168.1.100:5000/steer/45
  - Method: GET
  - Headers: Content-Type: application/json
         ↓
fetch() sends request over network
         ↓
Request reaches Raspberry Pi on port 5000
         ↓
Flask router matches /steer/<int:angle> pattern
```

### 3. Server Processing
```
Flask route handler steer(angle=45) executes:
         ↓
car.set_steering(45) called
         ↓
Sets CarSystem.steering_angle = 45
         ↓
Calls car.update()
         ↓
Calculates motor speeds based on gear & steering
         ↓
_set_raw_motors() updates GPIO pins
         ↓
PWM duty cycles set on GPIO 18 & 19
         ↓
Returns "OK" response
```

### 4. Hardware Response
```
GPIO pins control L298N motor driver:
  IN1/IN2 → Left motor direction & speed
  IN3/IN4 → Right motor direction & speed
  ENA/ENB → PWM duty cycle (speed control)
         ↓
Motors respond to speed changes
         ↓
Steering servo or motor adjust car direction
         ↓
User sees physical response
```

### 5. UI Update
```
fetch() receives "OK" response
         ↓
No error thrown
         ↓
React component continues normal state updates
         ↓
Dashboard shows steering angle, speed, etc.
         ↓
Loop repeats for next user input
```

## Complete Command Flow Example: Accelerating

```
1. USER PRESSES THROTTLE
   └─→ Pedals component detects press

2. FRONTEND PROCESSES
   └─→ handleThrottleChange(true) 
   └─→ setControlState({...prev, throttle: true})
   └─→ sendCommand("throttle", true)

3. HTTP REQUEST SENT
   └─→ fetch("http://192.168.1.100:5000/gas/on")

4. FLASK RECEIVES REQUEST
   └─→ @app.route("/gas/<state>")
   └─→ def gas(state): 
       └─→ car.set_gas(True)

5. MOTOR CONTROL UPDATES
   └─→ CarSystem.set_gas(True)
   └─→ is_gas_pressed = True
   └─→ update()
   └─→ base_speed = GEAR_SPEEDS[current_gear]  (e.g., 50)
   └─→ _set_raw_motors(50, 50, ...)

6. GPIO PINS UPDATE
   └─→ ENA PWM: 50% duty cycle
   └─→ ENB PWM: 50% duty cycle
   └─→ Both motors spin forward

7. SPEED INCREASE
   └─→ Motors accelerate car
   └─→ UI speedometer updates (local simulation)
   └─→ Next query: if still throttling, maintain speed

8. USER RELEASES THROTTLE
   └─→ handleThrottleChange(false)
   └─→ sendCommand("throttle", false)
   └─→ car.set_gas(False)
   └─→ Motors decelerate
   └─→ Car slows down
```

## Code Component Interactions

### Frontend (React/TypeScript)
```
App.tsx
  └─→ Index.tsx
       └─→ CockpitController.tsx (Main control hub)
           ├─→ Header.tsx (Connection status)
           ├─→ SteeringWheel.tsx (Steering input)
           │   └─→ handleAngleChange() 
           │       └─→ sendCommand("steering", angle)
           │
           ├─→ Pedals.tsx (Throttle/Brake input)
           │   ├─→ handleThrottleChange()
           │   │   └─→ sendCommand("throttle", bool)
           │   └─→ handleBrakeChange()
           │       └─→ sendCommand("brake", bool)
           │
           ├─→ GearShifter.tsx (Gear selection)
           │   └─→ handleGearChange()
           │       └─→ sendCommand("gear", string)
           │
           └─→ CarTelemetry.tsx (Display only)
               └─→ Shows speed, gear, angle
```

### Backend (Python/Flask)
```
main.py
  ├─→ Flask app setup
  │   ├─→ Static file serving (React build)
  │   └─→ CORS enabled
  │
  ├─→ Motor initialization
  │   └─→ car = CarSystem()
  │
  └─→ API Routes:
       ├─→ GET /steer/<angle>
       │   └─→ car.set_steering(angle)
       │
       ├─→ GET /gear/<g>
       │   └─→ car.set_gear(g)
       │
       ├─→ GET /gas/<state>
       │   └─→ car.set_gas(True/False)
       │
       └─→ GET /brake
           └─→ car.emergency_brake()

motor.py
  └─→ CarSystem class
       ├─→ __init__()
       │   ├─→ GPIO setup
       │   ├─→ PWM initialization
       │   └─→ State variables
       │
       ├─→ set_steering(angle)
       │   └─→ steering_angle = angle
       │       └─→ update()
       │
       ├─→ set_gear(gear)
       │   └─→ current_gear = gear
       │       └─→ update()
       │
       ├─→ set_gas(pressed)
       │   └─→ is_gas_pressed = pressed
       │       └─→ update()
       │
       ├─→ update()
       │   ├─→ Calculate base_speed from gear
       │   ├─→ Apply steering physics
       │   ├─→ Determine motor speeds
       │   └─→ _set_raw_motors(left, right, ...)
       │
       └─→ _set_raw_motors()
           ├─→ Set GPIO output levels
           └─→ Set PWM duty cycles
```

## Data Flow Diagram: Steering Example

```
┌─────────────────────────────────────────┐
│ User rotates steering wheel to -45°     │
└──────────────────┬──────────────────────┘
                   │
       ┌───────────▼──────────────┐
       │ SteeringWheel.tsx        │
       │ onAngleChange(-45)       │
       └───────────┬──────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ CockpitController.tsx                │
       │ handleAngleChange(-45)               │
       │ setControlState({...})               │
       │ sendCommand("steering", -45)         │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ Browser HTTP Layer                   │
       │ fetch("http://192.168.1.100:5000/   │
       │   steer/-45")                        │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ Network                              │
       │ HTTP GET request sent to Pi:5000     │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ Flask/main.py                        │
       │ @app.route("/steer/<int:angle>")     │
       │ steer(-45)                           │
       │ car.set_steering(-45)                │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ motor.py: CarSystem                  │
       │ set_steering(-45)                    │
       │ steering_angle = -45                 │
       │ update()                             │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ Physics Calculation                  │
       │ intensity = 45/90 = 0.5              │
       │ inner_wheel = 1.0 - (0.5 × 0.9) =   │
       │            = 1.0 - 0.45 = 0.55      │
       │ left_speed = 50 × 0.55 = 27.5       │
       │ right_speed = 50                     │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ _set_raw_motors(27.5, 50, ...)       │
       │ GPIO.output(IN1, IN2, IN3, IN4, ...) │
       │ pwm_a.ChangeDutyCycle(27.5)          │
       │ pwm_b.ChangeDutyCycle(50)            │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ L298N Motor Driver                   │
       │ Left motor: 27.5% speed              │
       │ Right motor: 50% speed               │
       │ → Car turns left!                    │
       └───────────┬──────────────────────────┘
                   │
       ┌───────────▼──────────────────────────┐
       │ Physical Response                    │
       │ Car steers left as commanded         │
       │ Both wheels move forward             │
       │ Inner wheel (left) slower            │
       │ Outer wheel (right) faster           │
       └───────────────────────────────────────┘
```

## Timing & Performance

### Typical Response Times
- **User Input → Browser Event:** <1ms
- **Browser → HTTP Request:** <1ms  
- **Network Latency (local):** 5-50ms
- **Flask Processing:** 1-5ms
- **GPIO Update:** <1ms
- **Motor Response:** 10-50ms (mechanical inertia)
- **Total End-to-End:** 30-150ms

### Update Frequency
- **Frontend Poll Rate:** Every user interaction
- **Speed Simulation:** Every 100ms (internal timer)
- **Motor Update Rate:** Immediate (synchronous)
- **Network Rate:** No auto-polling (event-driven)

## Steering Physics Explanation

The system uses "differential steering" - slowing the inner wheel during turns:

```
Straight (0°):          Left Turn (-45°):
┌─────────┐              ┌─────────┐
│ L: 50%  │              │ L: 27%  │  (inner wheel slowed)
│ R: 50%  │              │ R: 50%  │  (outer wheel maintains)
│ Speed   │              │ Turn    │
└─────────┘              └─────────┘

Formula:
intensity = |steering_angle| / 90
inner_factor = 1.0 - (intensity × 0.9)
inner_speed = base_speed × inner_factor
outer_speed = base_speed (unchanged)

At -90° (full left turn):
intensity = 1.0
inner_factor = 1.0 - 0.9 = 0.1
Left motor runs at 10% (minimum)
Right motor runs at 100% (maximum)
→ Tightest possible turn
```

This creates smooth, realistic steering without abrupt direction changes.

## GPIO Pin Mapping

```
Raspberry Pi GPIO (BCM)
    │
    ├─ 17 (IN1) ─→ L298N IN1 ─→ Left Motor Forward
    ├─ 27 (IN2) ─→ L298N IN2 ─→ Left Motor Backward
    ├─ 22 (IN3) ─→ L298N IN3 ─→ Right Motor Forward
    ├─ 23 (IN4) ─→ L298N IN4 ─→ Right Motor Backward
    ├─ 18 (ENA) ─→ L298N ENA ─→ Left Motor Speed (PWM)
    ├─ 19 (ENB) ─→ L298N ENB ─→ Right Motor Speed (PWM)
    │
    └─ Power
       ├─ 5V ─→ L298N +5V
       └─ GND ─→ L298N GND
```

## Error Handling

Current error handling:
```
Frontend:
  └─→ fetch(...).catch(err => console.error(...))
      │ Logs to browser console but continues
      │ No user notification of failure
      │ No retry logic

Backend:
  └─→ Flask returns "OK" regardless
      │ No error checking on GPIO operations
      │ No validation of parameters
      │ No exception handling
```

Future improvements could include:
- Status/telemetry responses with motor feedback
- Timeout detection and retry logic
- Parameter validation
- Error code responses
- Heartbeat/connection monitoring

## Summary

The system is a **simple request-response architecture**:
1. Browser sends HTTP GET request to Flask
2. Flask processes request immediately
3. Motor control updates happen synchronously
4. Response returns to browser
5. Process repeats for next user input

This design is **ideal for**: responsive, low-latency control
This design works best with: stable local network connections

For production/remote use, consider:
- WebSocket for real-time bidirectional communication
- Telemetry feedback from car sensors
- Connection reliability and error recovery
- Rate limiting and command queueing
