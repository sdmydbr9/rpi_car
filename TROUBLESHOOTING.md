# System Health Check & Troubleshooting

## Pre-Flight Checklist

Run through these checks before reporting issues:

### ✅ Python Dependencies
```bash
cd /home/pi/rpi_car
source venv/bin/activate
python3 -c "import flask; import flask_cors; print('✅ All packages installed')"
```

### ✅ React Build
```bash
ls -la f1-race-control/dist/
# Should show index.html and other files
```

### ✅ Server Startup
```bash
./run.sh
# Should see:
# ✅ UI is already built. Starting server...
# * Running on http://0.0.0.0:5000
```

### ✅ API Connectivity Test
From another terminal:
```bash
# Test steering endpoint
curl http://localhost:5000/steer/0

# Test gear endpoint  
curl http://localhost:5000/gear/1

# Test gas endpoint
curl http://localhost:5000/gas/on
```
All should return: `OK`

## Common Issues & Fixes

### Issue: "ModuleNotFoundError: No module named 'flask_cors'"

**Fix:**
```bash
source venv/bin/activate
pip install flask-cors
```

### Issue: "CORS policy: Cross-Origin Request Blocked"

**Check:** Open browser F12 → Console tab
- If you see CORS errors, flask_cors may not be installed
- Verify `from flask_cors import CORS` is in main.py (line 2)
- Verify `CORS(app)` is called in main.py (line 44)

### Issue: "Cannot GET /steer/0" or "Connection refused"

**Fix:**
1. Check Flask server is running: `ps aux | grep main.py`
2. Check port 5000 is available: `lsof -i :5000`
3. Make sure main.py line 82 has: `app.run(host='0.0.0.0', port=5000)`
4. Restart server: Kill process and run `./run.sh`

### Issue: "Failed to send steering: TypeError: Failed to fetch"

**This is a network issue:**
1. Verify Pi's IP: `hostname -I`
2. Ping from client: `ping <pi-ip>`
3. Make sure you're connecting to `http://` (not https)
4. Port must be `:5000` (not omitted)

### Issue: Web page loads but controls have no effect

**Step-by-step debug:**

1. **Check frontend is sending requests:**
   - Open browser F12
   - Go to Network tab
   - Move steering wheel
   - Should see GET request to `/steer/XX`
   - Check response: should be `OK`

2. **Check motor control is firing:**
   - Add debug print to motor.py:
   ```python
   def set_steering(self, angle):
       print(f"🎯 Steering to {angle}°")  # Add this line
       self.steering_angle = angle
       self.update()
   ```
   - Restart server
   - Move steering wheel in web UI
   - Check terminal for "🎯 Steering to X°" messages

3. **Check GPIO is working:**
   - Run: `gpio readall` (if wiringPi installed)
   - Or test with Python:
   ```python
   import RPi.GPIO as GPIO
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(17, GPIO.OUT)
   GPIO.output(17, True)
   print("✅ GPIO 17 high")
   GPIO.output(17, False)
   print("✅ GPIO 17 low")
   ```

4. **Verify motor connections:**
   - Check wiring to L298N motor driver
   - Verify pin numbers in motor.py match your setup
   - Test with multimeter if available

### Issue: "npm: command not found"

**Fix:** Install Node.js
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc
nvm install 18
npm --version  # Should show 9.x or higher
```

### Issue: Motors spin backwards or wrong direction

**Fix in motor.py:**
```python
# Edit the motor direction logic in update() method
# Change False/True combinations to reverse direction
# Example: Change (False, True) to (True, False) for left motor
```

### Issue: Steering sensitivity is wrong

**Fix in motor.py:**
```python
# Adjust steering intensity calculation around line 50:
intensity = abs(self.steering_angle) / 90.0  # Change 90 to different value
inner_wheel_factor = 1.0 - (intensity * 0.9)  # Change 0.9 to tune sharpness
```

### Issue: Car too slow or too fast

**Fix in motor.py:**
```python
# Edit GEAR_SPEEDS dictionary (around line 31):
self.GEAR_SPEEDS = {
    0: 0,      # Neutral
    1: 30,     # Increase for more speed
    2: 50,     # Adjust values (0-100 percent)
    3: 75,
    4: 100     # Sport mode
}
```

## Performance Monitoring

### Monitor Server Responsiveness
```bash
# Watch for errors in real-time
tail -f /var/log/your_app.log  # If you add logging

# Or run server in foreground to see all output
./run.sh
```

### Check Network Latency
From a different device:
```bash
# Measure round-trip time to commands
time curl http://<pi-ip>:5000/steer/0
# Should complete in <100ms on local network
```

### Monitor GPIO Activity
While server is running:
```bash
# Watch GPIO state changes (if gpio tools installed)
watch -n 0.1 gpio readall

# Or run Python directly
python3 << 'EOF'
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup([17, 27, 22, 23], GPIO.IN)
while True:
    print(f"IN1:{GPIO.input(17)} IN2:{GPIO.input(27)} IN3:{GPIO.input(22)} IN4:{GPIO.input(23)}")
    time.sleep(0.1)
EOF
```

## Remote Troubleshooting

If testing from another device and commands don't work:

1. **Can you reach the server?**
   ```bash
   curl http://<pi-ip>:5000/
   # Should return HTML content
   ```

2. **Can you reach the API?**
   ```bash
   curl http://<pi-ip>:5000/steer/0
   # Should return: OK
   ```

3. **Do you see the page load?**
   - Open http://<pi-ip>:5000 in browser
   - Should show racing dashboard

4. **Does the page connect?**
   - Click connect button
   - Enter Pi's IP
   - Click connect
   - Connection indicator should turn green

5. **Do requests get sent?**
   - Open F12 Developer Tools
   - Go to Network tab
   - Move steering wheel
   - Should see `/steer/XX` requests
   - Check if they get responses

## Restart Procedures

### Full restart:
```bash
# Kill the server
pkill -f main.py

# Rebuild React app (if changes made)
cd f1-race-control
npm run build
cd ..

# Clear Python cache
find . -type d -name __pycache__ -exec rm -r {} +

# Start fresh
./run.sh
```

### Restart just server (keep build):
```bash
pkill -f main.py
./run.sh
```

## Final Nuclear Option

If everything is broken:
```bash
cd /home/pi/rpi_car

# Remove old venv
rm -rf venv

# Create fresh environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Rebuild frontend
cd f1-race-control
npm install
npm run build
cd ..

# Start fresh
./run.sh
```

## Still Not Working?

1. **Collect diagnostic info:**
   ```bash
   echo "=== Pi Info ===" && uname -a
   echo "=== Python ===" && python3 --version
   echo "=== Node ===" && node --version && npm --version
   echo "=== Packages ===" && pip list | grep -i flask
   echo "=== Ports ===" && netstat -tuln | grep 5000
   echo "=== GPIO ===" && python3 -c "import RPi.GPIO; print('✅ GPIO available')" 2>&1
   ```

2. **Save Flask output:**
   ```bash
   ./run.sh 2>&1 | tee server.log
   # Then share server.log for debugging
   ```

3. **Check logs for errors:**
   - Review any error messages in the terminal running `main.py`
   - Check browser console (F12) for JavaScript errors
   - Look for CORS, network, or Python exceptions

## Support Resources

- **Flask Documentation:** https://flask.palletsprojects.com/
- **flask-cors Documentation:** https://flask-cors.readthedocs.io/
- **RPi.GPIO Documentation:** https://pypi.org/project/RPi.GPIO/
- **React Troubleshooting:** https://react.dev/

Remember: The most common issues are:
1. ❌ flask-cors not installed
2. ❌ Wrong IP address or port
3. ❌ GPIO permissions (run with sudo)
4. ❌ Motor wiring incorrect
