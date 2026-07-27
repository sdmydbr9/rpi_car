# Raspberry Pi Car — Manual Control Only

This repository contains a low-latency, human-operated controller for a
2WD Ackermann car:

```
Browser / mobile ─┐
                  ├─ Socket.IO direct controls ─> Raspberry Pi ─> UART ─> Pico
Pi gamepad ───────┘
```

There is no autonomous driving, camera stream, telemetry stream, odometry,
object detection, ROS, AI narration, encoder PID, or obstacle avoidance.
The ADS1115 battery channel is read only when a user explicitly requests it
while the car is disarmed.

## Safety model

- The car starts disarmed and connecting a client never releases e-stop.
- Only one remote client can own control.
- A connected Pi gamepad always takes priority and disarms remote control.
- Browser/mobile clients send full drive state every 100 ms only while moving.
- An active brake command is refreshed at the same 100 ms interval.
- The Pi stops and disarms a moving remote car after 300 ms without input.
- The Pico independently stops after 300 ms without a drive command.
- Forward/reverse changes must pass through zero throttle. The Pico adds a
  150 ms direction dwell as a second guard.
- E-stop is latched in both the Pi state and Pico firmware. Any remote can
  engage it; only the priority controller can reset it while neutral.
- Requested throttle `0–100%` maps to a maximum motor PWM of 95%.

This is still a real moving vehicle. Test with the drive wheels lifted clear
before putting the car on the ground.

## Retained hardware

| Component | Pico connection |
| --- | --- |
| Pi UART TX/RX | Pico GP1 RX / GP0 TX, 115200 baud |
| Left motor PWM/direction | GP10 / GP17 / GP12 |
| Right motor PWM/direction | GP16 / GP13 / GP14 |
| Ackermann steering servo | GP15 |
| ADS1115 battery input | I2C GP8 SDA / GP9 SCL, address `0x48`, channel A0 |

Default steering pulse widths are 940 µs left, 1440 µs center, and 2150 µs
right. They can be changed only while disarmed.

## Transfer and installation

1. Copy the repository to the Raspberry Pi, normally `/home/pi/rpi_car`.
2. Install the Pico firmware:
   - Open `scripts/firmware/pico_manual_controller.py` in Thonny.
   - Save it on the Pico as `main.py`.
   - Power-cycle the Pico.
3. Enable the Pi UART and disable the serial login console.
4. Install the Pi and web dependencies:

   ```bash
   cd /home/pi/rpi_car
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   npm install
   npm run build
   ```

   `npm install` creates fresh lockfiles on the target system; generated
   lockfiles were intentionally not committed after the dependency reduction.
5. Add the service user to the UART and input-device groups:

   ```bash
   sudo usermod -aG dialout,input "$USER"
   ```

   Log out and back in afterward.
6. Start the server:

   ```bash
   ./run.sh
   ```

7. Open `http://<pi-address>:5000`, take control, select F or R, arm, and use
   the spring-return throttle and steering controls.

See [TRANSFER_CHECKLIST.md](TRANSFER_CHECKLIST.md) before the first drive.

## Configuration

Environment variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `RC_PICO_PORT` | `/dev/ttyS0` | Pi UART device |
| `RC_BATTERY_DIVIDER_RATIO` | `5.0` | Convert ADS1115 A0 voltage to pack voltage |
| `RC_HOTSPOT_PROFILE` | `CarHotspot` | NetworkManager hotspot profile |
| `RC_WIFI_PROFILE` | `wifi2` | NetworkManager Wi-Fi profile |

The battery value is informational. Missing, invalid, high, or low readings
do not limit or stop the car.

Steering calibration is persisted in `.steering_config.json`.

## Manual controls

### Browser and mobile

- Take Control / Release Control
- Arm / Disarm
- Forward / Neutral / Reverse
- Proportional throttle (`0–100%`)
- Proportional steering (`-50°…+50°`)
- Hold Brake
- Latched Emergency Stop / Reset
- On-demand battery reading

Throttle and steering return to zero when released. Hiding or closing a client
sends zero and disarms; the two watchdogs remain the final fallback.

### Pi-connected gamepad

| Input | Action |
| --- | --- |
| Left stick Y | Signed proportional throttle; forward/up, reverse/down |
| Right stick X | Proportional steering |
| L3 or R3 | Hold brake |
| Start | Arm/disarm |
| Select twice within 0.6 s | Immediate stop and disarm |
| LT or RT + X | Engage e-stop; repeat at neutral to reset |

When a gamepad first produces input, remote ownership is revoked and the car
is stopped. When it disconnects, the car is stopped and disarmed again.
The throttle and brake must be released before Start can arm the car.

## UART protocol

The Pico sends no periodic data and does not acknowledge normal drive packets.

| Command | Meaning |
| --- | --- |
| `D:F,47,1440` | Forward, 47% PWM, steering pulse 1440 µs |
| `D:R,30,1200` | Reverse drive |
| `D:N,0,1440` | Neutral/steering-only update |
| `B` | Active brake; refresh every 100 ms while held |
| `S` | Coast stop |
| `E` | Latch e-stop |
| `ERST` | Reset e-stop when stopped |
| `SC:940,1440,2150` | Steering calibration |
| `PING` | Reply once with `PONG` |
| `BAT?` | Reply once with `BAT:<adc_mv>` or `BAT:ERR` |

## Deferred validation

No tests or builds were run on the development machine used for this rewrite.
After transfer, run:

```bash
source venv/bin/activate
python -m unittest discover -s tests -v
npm install
npm test
npm run lint
npm run build
cd app
npm install
npm run typecheck
```

Then run the stationary hardware utility:

```bash
cd /home/pi/rpi_car
source venv/bin/activate
python scripts/diagnostics/manual_control_smoke.py
python scripts/diagnostics/manual_control_smoke.py --allow-motor-test
```

The second command requires a typed confirmation and must be run with the
drive wheels lifted clear.

For interactive server-mediated checks without opening the UART separately:

```bash
python scripts/diagnostics/interactive_control_test.py
```

It offers status, manual battery refresh, centering/sweeping steering, and a
guarded motor pulse. The motor option requires typing `LIFTED`, is capped at
20% throttle for two seconds, and always sends neutral/disarm on exit.
