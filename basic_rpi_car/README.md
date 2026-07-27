# basic_rpi_car

`basic_rpi_car` is a standalone, gamepad-only control system for the
Raspberry Pi car. It has no web server, network controller, camera,
autonomous driving, or telemetry stream.

The Raspberry Pi reads the connected Linux gamepad and sends command packets
over UART to the Pico. The Pico directly controls the two motors and Ackermann
steering servo.

```text
Gamepad -> Raspberry Pi main.py -> UART -> Pico -> motors and steering
```

The car starts disarmed. Always perform the first tests with the drive wheels
lifted clear of the ground.

## Hardware configuration

| Component | Pico connection |
| --- | --- |
| Pi UART TX/RX | Pico GP1 RX / GP0 TX, 115200 baud |
| Left motor PWM/direction | GP10 / GP17 / GP12 |
| Right motor PWM/direction | GP16 / GP13 / GP14 |
| Steering servo | GP15 |
| ADS1115 battery input | I2C GP8 SDA / GP9 SCL, address `0x48`, A0 |

The motor PWM frequency is 1 kHz and output is capped at 95%. The default
steering pulses are 940 us left, 1440 us center, and 2150 us right.

## Install the Pico firmware

1. Open `pico_firmware.py` in Thonny.
2. Connect to the Pico interpreter.
3. Save the file on the Pico as `main.py`.
4. Power-cycle the Pico.

The firmware immediately coasts the motors and centers the steering at boot.
It independently stops motor output if an active drive/brake command is not
refreshed within 300 ms.

## Install on the Raspberry Pi

From this directory:

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

Enable the Pi UART, disable the serial login console, and give the current
user access to serial and input devices:

```bash
sudo usermod -aG dialout,input "$USER"
```

Log out and back in after changing group membership.

## Run

```bash
cd basic_rpi_car
source venv/bin/activate
python3 main.py
```

The controller waits for both devices, starts disarmed, and reconnects after
a temporary disconnection. Press Start with the throttle and brake released
to arm.

The default Pico UART is `/dev/ttyS0`. Override it with either:

```bash
python3 main.py --port /dev/serial0
RC_PICO_PORT=/dev/serial0 python3 main.py
```

## Gamepad controls

| Input | Action |
| --- | --- |
| Left stick Y | Proportional throttle: up/forward, down/reverse |
| Right stick X | Proportional steering |
| L3 or R3 | Hold active brake |
| Start | Arm or disarm |
| Select twice within 0.6 seconds | Immediate stop and disarm |
| LT or RT + X | Latch emergency stop |
| LT or RT + X again while neutral | Reset emergency stop; remain disarmed |

The sticks use an 8% center deadzone. A forward/reverse change is forced
through zero output, and the Pico adds a 150 ms direction dwell.

Disconnecting the gamepad stops and disarms the car. `Ctrl+C`, `SIGTERM`, and
normal cleanup send the latched Pico emergency-stop command. After restarting
the Pi process following an emergency-stop shutdown, use the e-stop combo to
engage/synchronize the local latch and then repeat it while neutral to reset.

## Steering calibration

Create `.steering_config.json` in this directory if the default steering
pulses do not match the existing calibration:

```json
{
  "left_pw": 940,
  "center_pw": 1440,
  "right_pw": 2150
}
```

The values must satisfy:

```text
500 <= left_pw < center_pw < right_pw <= 2500
```

Use another file with `--calibration PATH` or the `RC_STEERING_CONFIG`
environment variable. Invalid or missing configuration falls back to the
defaults and never expands the allowed 500-2500 us servo range.

## Stationary diagnostics

With the motors stopped:

```bash
python3 main.py --ping
python3 main.py --battery
```

`--battery` reports the ADS1115 A0 voltage and the estimated pack voltage.
The divider ratio defaults to `5.0` and can be changed:

```bash
RC_BATTERY_DIVIDER_RATIO=4.7 python3 main.py --battery
```

The Pico sends no periodic data. It replies only to `PING` and `BAT?`.

## Test

Run the hardware-free tests:

```bash
python3 -m unittest discover -s tests -v
```

Then, with the drive wheels lifted:

1. Confirm the car remains stopped until Start is pressed at neutral.
2. Check low-throttle forward, reverse, and the full steering range.
3. Check L3 and R3 braking independently.
4. Confirm a direct forward-to-reverse stick movement stops before reversing.
5. Confirm double-Select stops and disarms.
6. Confirm the trigger + X e-stop latches, resets only at neutral, and leaves
   the car disarmed.
7. Unplug the gamepad while moving and confirm an immediate stop/disarm.
8. Disconnect the Pi-to-Pico UART TX line at low throttle and confirm the Pico
   watchdog removes motor output within 300 ms.
9. Terminate `main.py` and confirm the motors remain stopped.
