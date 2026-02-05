# IR Obstacle Sensors - Installation & Implementation

## Hardware Configuration

### Front IR Sensor
- **VCC** → Pin 1 (3.3V Power)
- **GND** → Pin 9 (Ground)
- **OUT** → Pin 29 (GPIO 5)

### Back IR Sensor
- **VCC** → Pin 17 (3.3V Power)
- **GND** → Pin 25 (Ground)
- **OUT** → Pin 31 (GPIO 6)

## Software Changes

### Files Updated
1. **main.py** - Flask server with obstacle detection logic
2. **motor.py** - CarSystem class with IR sensor support

### Key Features Implemented

#### 1. GPIO Pin Configuration
- Added `FRONT_IR = 5` and `BACK_IR = 6` pin definitions
- Configured pins as inputs for sensor reading

#### 2. Obstacle Detection Logic
- **IR Sensor Reading**: Active LOW (0 = obstacle, 1 = no obstacle)
- Reading is inverted in code: `not GPIO.input()` so True = obstacle detected
- Sensors are checked every physics loop cycle (~50ms)

#### 3. Movement Blocking
- **Forward Movement**: Blocked when `front_obstacle` is True and car is in gear (1, 2, or 3)
- **Reverse Movement**: Blocked when `back_obstacle` is True and car is in Reverse (R)
- Motor commands are overridden to prevent movement

#### 4. Telemetry Integration
- New telemetry endpoint returns:
  - `"front_obstacle": true/false`
  - `"back_obstacle": true/false`
- Allows UI to display sensor status

#### 5. Console Logging
- ⚠️ `FRONT OBSTACLE DETECTED!` - When front sensor triggers
- ⚠️ `BACK OBSTACLE DETECTED!` - When back sensor triggers
- 🛑 `FORWARD BLOCKED by front obstacle!` - When forward movement is prevented
- 🛑 `REVERSE BLOCKED by back obstacle!` - When reverse movement is prevented

## How It Works

1. **Continuous Monitoring**: IR sensors are read every 50ms in the physics loop
2. **Priority System**: Obstacle detection is checked before braking logic
3. **Graceful Stopping**: When an obstacle is detected during forward/reverse, the car automatically stops
4. **No Manual Override**: Once an obstacle is detected, the throttle is disabled until cleared

## Testing

To test the sensors:
```bash
# SSH into Raspberry Pi
ssh pi@192.168.x.x

# Navigate to project
cd ~/rpi_car

# Run the main server
python3 main.py

# Check the console output for obstacle detection messages
```

To manually test sensor input:
```bash
# Check front IR sensor (GPIO 5)
gpio -1 mode 5 in
gpio -1 read 5

# Check back IR sensor (GPIO 6)
gpio -1 mode 6 in
gpio -1 read 6
```

Values should be:
- `1` = No obstacle detected
- `0` = Obstacle detected

## Integration with UI

The telemetry endpoint (`/telemetry`) now includes obstacle sensor data. The React UI can:
- Display obstacle warnings
- Show sensor status indicators
- Alert driver when obstacles are detected

Update your React components to show these values from the telemetry response.
