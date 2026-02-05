# Raspberry Pi Car Control System

A complete control system for a Raspberry Pi-based remote control car with a web-based cockpit dashboard.

## 📋 Project Structure

- **main.py** - Flask web server that serves the UI and provides REST API for car control
- **motor.py** - Motor control library using RPi.GPIO for hardware control
- **f1-race-control/** - React/TypeScript web application (cockpit dashboard)

## 🔧 Setup

### Prerequisites

- Raspberry Pi with GPIO pins available
- Python 3.7+
- Node.js 18+ (for building the React app)
- npm or bun

### Installation

1. **Install Python dependencies:**
   ```bash
   cd /home/pi/rpi_car
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

2. **Install and build the Node.js application:**
   ```bash
   cd f1-race-control
   npm install
   npm run build
   cd ..
   ```

## 🚀 Running the Server

### Option 1: Using the startup script
```bash
cd /home/pi/rpi_car
chmod +x run.sh
./run.sh
```

### Option 2: Manual startup
```bash
cd /home/pi/rpi_car
source venv/bin/activate
python3 main.py
```

The server will:
1. Check if the React app is built (and build it automatically if needed)
2. Start the Flask server on `http://0.0.0.0:5000`
3. Be accessible from any device on your network at `http://<raspberry-pi-ip>:5000`

## 🎮 Using the Web Dashboard

1. Open your browser and navigate to the Raspberry Pi's IP address on port 5000
2. Click the connection button in the top-right corner
3. Enter the Raspberry Pi's IP address (e.g., `192.168.1.100`)
4. Click "Connect"
5. Use the controls:
   - **Steering Wheel** - Rotate to control steering (-90 to +90 degrees)
   - **Gear Shifter** - Select gear: R, N, 1, 2, 3, S
   - **Pedals** - Throttle (right) and Brake (left)

## 🔌 Hardware Configuration

The `motor.py` file uses the following GPIO pins by default:

**Motor Control (L298N):**
- **IN1, IN2** (GPIO 17, 27) - Left motor direction
- **IN3, IN4** (GPIO 22, 23) - Right motor direction  
- **ENA, ENB** (GPIO 18, 19) - Motor speed (PWM at 1000Hz)

**Gear Mapping:**
- `0` - Neutral/Brake
- `1` - Gear 1 (30% max speed)
- `2` - Gear 2 (50% max speed)
- `3` - Gear 3 (75% max speed)
- `4` - Sport (100% max speed)

**Steering Physics:**
- Smooth steering with reduced inner wheel speed during turns
- Steering range: -90° (left) to +90° (right)

## 📡 API Endpoints

The Flask server provides the following REST endpoints:

- **GET `/steer/<angle>`** - Set steering angle (-90 to 90)
- **GET `/gear/<gear>`** - Set gear (0-4)
- **GET `/gas/<state>`** - Control throttle (on/off)
- **GET `/brake`** - Emergency brake

Example requests:
```bash
# Turn left
curl http://localhost:5000/steer/-45

# Select gear 2
curl http://localhost:5000/gear/2

# Throttle on
curl http://localhost:5000/gas/on

# Emergency brake
curl http://localhost:5000/brake
```

## 🐛 Troubleshooting

### Controls not responding
1. Ensure the Flask server is running: `ps aux | grep python`
2. Check the browser console for network errors (F12)
3. Verify the Raspberry Pi IP address is correct
4. Check CORS is enabled (should be in main.py)

### Server won't start
1. Check that port 5000 is not in use: `lsof -i :5000`
2. Ensure the UI is built: `npm run build` in the f1-race-control directory
3. Verify Python packages are installed: `pip list | grep -E "flask|cors"`

### Motors not responding
1. Check GPIO pin configuration in motor.py
2. Verify wiring to motor controller
3. Test GPIO directly with `gpio readall` command
4. Check for error messages in the Flask server console

## 📝 License

This project is for educational and personal use.
