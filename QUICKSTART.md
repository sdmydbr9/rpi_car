# Quick Start Guide

## First Time Setup (5-10 minutes)

### 1. Clone/Prepare the project
```bash
cd /home/pi/rpi_car
```

### 2. Install Python dependencies
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Install and build the web app
```bash
cd f1-race-control
npm install
npm run build
cd ..
```

## Running the System

### Start the server:
```bash
./run.sh
```

Or manually:
```bash
source venv/bin/activate
python3 main.py
```

The server will:
- ✅ Automatically build the React app if needed
- ✅ Start on port 5000
- ✅ Be accessible at `http://<your-pi-ip>:5000`

## Connecting from Another Device

1. **Find your Pi's IP:**
   ```bash
   hostname -I
   ```

2. **Open browser on another device:**
   - Navigate to: `http://<pi-ip>:5000`
   - Example: `http://192.168.1.100:5000`

3. **Connect in the web app:**
   - Click the connection icon (top-right)
   - Enter the IP address (same as above)
   - Click "Connect"

## What's Fixed

✅ **CORS enabled** - Browser can now communicate with Flask backend
✅ **API endpoints implemented** - Frontend now actually sends commands to the car
✅ **Command mapping** - Steering, gear, throttle, and brake are all mapped correctly
✅ **Dependencies installed** - flask-cors is available

## Testing the Connection

Open browser developer console (F12) and check:
1. No CORS errors in the Console tab
2. Network requests appear in the Network tab when you interact with controls
3. Check the Flask server logs for API calls

## Debugging

If controls still don't work:

1. **Check server is running:**
   ```bash
   curl http://localhost:5000/steer/0
   ```
   Should return: `OK`

2. **Check from another device:**
   ```bash
   curl http://<pi-ip>:5000/steer/0
   ```

3. **Monitor server logs:**
   Watch the terminal running `main.py` for any errors

4. **Check browser console:**
   Press F12 → Console tab for JavaScript errors

## Next Steps

- Verify GPIO connections to motors
- Test motor responses manually: `curl http://localhost:5000/gas/on`
- Adjust motor speed values in `motor.py` GEAR_SPEEDS if needed
- Add logging to track API calls in `main.py`
