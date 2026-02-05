# Changes Made to Fix Control Communication

## Summary
The web controls were not responding because there was no communication link between the React frontend and the Python backend. The frontend was logging commands to console instead of sending them to the Flask API.

## Files Modified

### 1. **f1-race-control/src/components/cockpit/CockpitController.tsx**
**Problem:** The `sendCommand` function had a TODO comment and was only logging to console
**Solution:** Implemented actual HTTP GET requests to Flask API endpoints

Changes:
- Maps command names to Flask endpoint paths
- Implements gear number conversion (R→0, N→0, 1→1, etc.)
- Throttle and brake control properly routed to `/gas` endpoint
- Sends requests to `http://{serverIp}:5000/endpoint`

Example:
```typescript
// Before: Just logged to console
console.log(`[${serverIp}] ${command}:`, value);

// After: Sends actual API request
fetch(`http://${serverIp}:5000/steer/${Math.round(value)}`, {
  method: 'GET',
  headers: { 'Content-Type': 'application/json' },
}).catch(err => console.error(`Failed to send steering:`, err));
```

### 2. **main.py**
**Problem:** No CORS support prevented browser from making cross-origin requests
**Solution:** Added flask-cors for cross-origin request handling

Changes:
- Added `from flask_cors import CORS` import
- Enabled CORS with `CORS(app)` after Flask app creation
- Now accepts requests from any origin (browser on different device/port)

## Files Created

### 3. **requirements.txt**
Lists Python dependencies for easy installation:
- flask==3.1.2
- flask-cors==6.0.2

### 4. **run.sh**
Convenient startup script that:
- Activates virtual environment
- Runs the Flask server
- Makes it easy to restart with one command

### 5. **README.md**
Comprehensive documentation including:
- Project structure overview
- Setup instructions
- Usage guide
- API endpoint reference
- Hardware pin configuration
- Troubleshooting guide

### 6. **QUICKSTART.md**
Quick reference guide for:
- First-time setup (step-by-step)
- Running the system
- Connection instructions
- Testing the connection
- Debugging tips

## How It Works Now

1. **User interacts with web dashboard** (moves steering wheel, changes gear, etc.)
2. **React component sends command** via `sendCommand()` function
3. **HTTP request sent to Flask API** at `http://{pi-ip}:5000/{endpoint}`
4. **Flask backend processes request** and calls appropriate motor control method
5. **CarSystem.update()** applies the control to GPIO pins
6. **Motors respond** to commands

## API Endpoint Mapping

| Frontend Action | Command Sent | Flask Endpoint | Motor Method |
|---|---|---|---|
| Steering wheel turn | `steering` + angle | `/steer/{angle}` | `car.set_steering()` |
| Gear change | `gear` + letter | `/gear/{num}` | `car.set_gear()` |
| Throttle pressed | `throttle` + true | `/gas/on` | `car.set_gas(True)` |
| Throttle released | `throttle` + false | `/gas/off` | `car.set_gas(False)` |
| Brake pressed | `brake` + true | `/brake` | `car.emergency_brake()` |

## Testing

To verify the fix works:

1. Start server: `./run.sh`
2. Open browser: `http://localhost:5000`
3. Click connect button, enter IP
4. Move steering wheel
5. Check Flask console for: `GET /steer/XX 200 OK`
6. Check browser Network tab (F12) for outgoing requests

## Installation of Dependencies

The setup includes:
```bash
# Python packages
pip install flask flask-cors

# Virtual environment for isolation
python3 -m venv venv
source venv/bin/activate
```

## Remaining Considerations

1. **GPIO Access** - May need to run as sudo if permissions prevent GPIO access
2. **Network Security** - CORS is open to all origins (consider restricting in production)
3. **Motor Testing** - Verify GPIO pins match actual wiring in motor.py
4. **Development** - Frontend dev server can be run with `npm run dev` in f1-race-control folder for faster iteration

## Summary of Fixes

✅ Frontend now sends real API requests (not just console logs)
✅ Flask backend has CORS enabled to accept browser requests
✅ All command types properly mapped to Flask endpoints
✅ Virtual environment set up with required dependencies
✅ Comprehensive documentation and startup scripts provided
