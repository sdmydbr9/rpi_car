# 🏎️ Raspberry Pi RC Car Control System

A sophisticated autonomous and manual control system for a Raspberry Pi-powered RC car with real-time telemetry, computer vision, and a modern web interface.

## 🎯 Features

### 🚗 Control Modes
- **Manual Control**: Precision steering, throttle, and braking via web interface
- **Autonomous Mode**: AI-powered obstacle avoidance with finite state machine
- **Emergency Brake**: Instant stop functionality for safety

### 🧠 Autonomous Driving
- 7-state finite state machine (FSM) for intelligent navigation
- Dual-direction protection (front + rear sonar)
- Adaptive speed control based on proximity
- Escalating escape maneuvers (reverse, pivot, U-turn)
- Stuck detection and recovery

### 👁️ Computer Vision
- Real-time object detection using MobileNetSSD
- Obstacle path prediction
- Visual telemetry with FPS monitoring
- Camera feed streaming

### 📡 Sensors
- **Front Sonar**: HC-SR04 ultrasonic (obstacle detection)
- **Rear Sonar**: HC-SR04 ultrasonic (reverse safety)
- **IR Sensors**: Left/Right obstacle detection
- **System Monitoring**: CPU temperature, clock speeds, RPM

### 🎮 User Interface
- Modern F1-inspired cockpit design
- Real-time telemetry dashboard
- Speedometer, tachometer, and gauges
- Camera feed with object detection overlay
- Autopilot status visualization
- Service light for sensor health
- Responsive landscape-optimized layout

## 🏗️ Architecture

```
┌─────────────────┐         WebSocket/HTTP         ┌──────────────────┐
│  React Frontend │ ◄─────────────────────────────► │  Flask Backend   │
│   (Vite + TS)   │                                 │   (Python 3)     │
└─────────────────┘                                 └──────────────────┘
                                                             │
                                                             ▼
                                                    ┌──────────────────┐
                                                    │  Hardware Layer  │
                                                    │  - GPIO Motors   │
                                                    │  - Sensors       │
                                                    │  - Camera        │
                                                    └──────────────────┘
```

## 📋 Prerequisites

### Hardware
- Raspberry Pi 3B+ or newer
- L298N Motor Driver
- HC-SR04 Ultrasonic Sensors (x2)
- IR Obstacle Sensors (x2)
- DC Motors (x2)
- Pi Camera or USB Camera
- RC Car Chassis

### Software
- Raspberry Pi OS (Bullseye or newer)
- Python 3.8+
- Node.js 18+
- npm 9+

## 🚀 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/sdmydbr9/rpi_car.git
cd rpi_car
```

### 2. Backend Setup (Python)
```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Frontend Setup (Node.js)
```bash
# Install Node.js dependencies
npm install

# Build the frontend
npm run build
```

### 4. Hardware Configuration

Edit `main.py` to configure GPIO pins for your wiring:

```python
# Motor Pins (BCM Numbering)
IN1 = 17  # Left Motor Direction A
IN2 = 27  # Left Motor Direction B
IN3 = 22  # Right Motor Direction A
IN4 = 23  # Right Motor Direction B
ENA = 18  # Left Motor Speed (PWM)
ENB = 19  # Right Motor Speed (PWM)

# Sonar Pins
SONAR_TRIG = 25  # Front Sonar Trigger
SONAR_ECHO = 24  # Front Sonar Echo

# IR Sensor Pins
LEFT_IR = 5   # Left IR Sensor
RIGHT_IR = 6  # Right IR Sensor
```

## 🏃 Running the Application

### Development Mode

#### Backend Only (Testing without hardware)
```bash
source venv/bin/activate
python3 main.py
```

#### Frontend Development Server
```bash
npm run dev
```

### Production Mode (on Raspberry Pi)

```bash
# Make run script executable
chmod +x run.sh

# Start the application
./run.sh
```

The server will start on `http://0.0.0.0:5000`

Access the web interface at:
- Local: `http://localhost:5000`
- Network: `http://<raspberry-pi-ip>:5000`

## 🎮 Usage

### Manual Control
1. Connect to the web interface
2. Select gear (D for Drive, R for Reverse, N for Neutral)
3. Use on-screen controls:
   - **Steering Wheel**: Turn left/right
   - **Throttle Pedal**: Accelerate
   - **Brake Pedal**: Slow down/stop

### Autonomous Mode
1. Enable sensors (IR and Sonar)
2. Toggle **Autopilot** switch
3. Car will navigate autonomously, avoiding obstacles

### Emergency Stop
Press the **E-BRAKE** button at any time to immediately stop the car.

## 🛠️ Development

### Available Scripts

#### Frontend
```bash
npm run dev          # Start Vite dev server
npm run build        # Build for production
npm run preview      # Preview production build
npm run lint         # Run ESLint
npm test             # Run Vitest tests
npm run test:watch   # Run tests in watch mode
```

#### Backend
```bash
python3 main.py              # Start Flask server
python3 systemcheck.py       # Test hardware connections
python3 simple_test.py       # Basic motor test
python3 test_autopilot_bug.py  # Autopilot unit tests
```

### Project Structure
```
rpi_car/
├── src/                    # React frontend
│   ├── components/         # UI components
│   │   ├── cockpit/       # Cockpit control components
│   │   └── ui/            # shadcn/ui components
│   ├── hooks/             # Custom React hooks
│   ├── lib/               # Utilities and Socket.IO client
│   └── pages/             # Route pages
├── main.py                # Flask server & main application
├── autopilot.py           # Autonomous driving FSM
├── motor.py               # Motor control system
├── sensors.py             # Sensor interface
├── vision.py              # Computer vision module
├── requirements.txt       # Python dependencies
├── package.json           # Node.js dependencies
└── README.md             # This file
```

## 🔧 Configuration

### Tuning Constants

Adjust autopilot behavior in `autopilot.py`:
- `FRONT_CRITICAL_CM`: Emergency brake threshold (default: 5 cm)
- `DANGER_CM`: Obstacle avoidance trigger (default: 40 cm)
- `MAX_SPEED`: Maximum cruise speed PWM (default: 80%)
- `MIN_SPEED`: Minimum cruise speed PWM (default: 30%)

Or adjust via the **Settings** dialog in the web interface.

### Camera Configuration

Edit `vision.py` to change camera source:
```python
camera = cv2.VideoCapture(0)  # 0 for USB camera, or RTSP URL
```

## 🧪 Testing

### Hardware Tests
```bash
# Test GPIO and sensors
python3 systemcheck.py

# Test motor control
python3 simple_test.py

# Test autopilot logic
python3 test_autopilot_bug.py
```

### Frontend Tests
```bash
npm test
```

## 🐛 Troubleshooting

### GPIO Errors
- Ensure you're running with `sudo` on Raspberry Pi
- Check GPIO pin connections match `main.py` configuration
- Verify BCM numbering mode is being used

### Camera Not Working
- Check camera is enabled: `sudo raspi-config` → Interface Options → Camera
- Test camera: `vcgencmd get_camera`
- Verify camera module in `/dev/video0`

### Connection Issues
- Check firewall settings allow port 5000
- Verify Raspberry Pi IP address
- Ensure both devices are on same network

### Build Errors
- Run `npm install` to ensure all dependencies are installed
- Clear cache: `rm -rf node_modules dist && npm install`
- Verify Node.js version: `node --version` (should be 18+)

## 📝 License

This project is provided as-is for educational purposes.

## 🙏 Credits

- **UI Framework**: React + Vite + TypeScript
- **UI Components**: shadcn/ui (Radix UI)
- **Styling**: Tailwind CSS
- **Backend**: Flask + Flask-SocketIO
- **Computer Vision**: OpenCV + MobileNetSSD
- **Hardware**: Raspberry Pi + L298N Motor Driver

## 🤝 Contributing

This is a personal project, but suggestions and improvements are welcome via issues.

## 📞 Support

For issues or questions, please open an issue on the GitHub repository.
