# basic_rpi_car

`basic_rpi_car` is a standalone, gamepad-only control system for the
Raspberry Pi car. It has no web server, network controller, camera,
autonomous driving, or telemetry stream.

The Raspberry Pi reads the connected Linux gamepad, sends motor commands over
UART to the Pico, and drives the Ackermann steering servo directly.

```text
Gamepad -> Raspberry Pi main.py -> UART -> Pico -> motors
                                -> GPIO12 -> steering servo
```

The car starts disarmed. Always perform the first tests with the drive wheels
lifted clear of the ground.

## Hardware configuration

| Component | Pico connection |
| --- | --- |
| Pi UART TX/RX | Pico GP1 RX / GP0 TX, 115200 baud |
| Left motor PWM/direction | GP10 / GP17 / GP12 |
| Right motor PWM/direction | GP16 / GP13 / GP14 |
| Steering servo signal | Pi GPIO12, physical pin 32 |
| ADS1115 battery input | I2C GP8 SDA / GP9 SCL, address `0x48`, A0 |

The motor PWM frequency is 1 kHz and output is capped at 95%. The default
Pi GPIO12 steering pulses are 940 us left, 1440 us center, and 2150 us right.

## Install the Pico firmware

1. Open `pico_firmware.py` in Thonny.
2. Connect to the Pico interpreter.
3. Save the file on the Pico as `main.py`.
4. Power-cycle the Pico.

The firmware immediately coasts the motors at boot and independently stops
motor output if an active drive/brake command is not refreshed within 300 ms.
Its former GP15 steering output is unused; moving steering to Pi GPIO12 does
not require reflashing the Pico.

## Install on the Raspberry Pi

From this directory:

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

Install and start the Pi's hardware-timed servo daemon once:

```bash
sudo apt install pigpio
sudo systemctl enable --now pigpiod
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

At connection time, `main.py` reads the gamepad's real Linux axis ranges
instead of guessing from individual events. Steering uses exactly one axis:
`ABS_RX` when available, otherwise `ABS_Z`. If the fallback axis has a stale
off-center startup value, the controller waits for one centered live event
before it can arm. This supports generic pads that expose a dummy `ABS_RX`
axis while preventing an unconfirmed trigger-like value from steering.
For an unusual controller whose right stick really is `ABS_Z`, override the
automatic choice:

```bash
python3 main.py --steering-axis ABS_Z
```

The car will not arm if the throttle and steering axes cannot be identified
safely. The startup log records the controller name, device path, selected
axes, ranges, current values, and deadzones.

Disconnecting the gamepad stops and disarms the car. `Ctrl+C`, `SIGTERM`, and
normal cleanup send the latched Pico emergency-stop command. After restarting
the Pi process following an emergency-stop shutdown, use the e-stop combo to
engage/synchronize the local latch and then repeat it while neutral to reset.

## Diagnostic logging

Normal runs show concise status on the console and write detailed `DEBUG`
records to `basic_rpi_car.log` beside `main.py`. The log rotates at 5 MB and
keeps three backups. Use another location or show the detailed records on the
console with:

```bash
python3 main.py --log-file /var/log/basic_rpi_car.log
python3 main.py --debug
```

Steering records share a sequence number from the raw gamepad event through
the requested servo pulse and GPIO12 readback. Pigpio output is also checked
once per second while armed, including when the motors are stationary. A
mismatch is corrected and verified; if the corrective command and its one
retry both fail, the motors stop and the car disarms.

Use the records around a failure to narrow down the cause:

- No `event=steering_input` when the stick moves means the problem is in the
  controller, its selected axis, or the input connection.
- A steering input without a matching successful `event=steering_apply`
  means the pigpio daemon or GPIO12 command path failed.
- A matching `requested_us` and `readback_us` while the wheels do not move
  means software delivered the pulse; inspect servo power, common ground,
  signal integrity, and mechanical binding.

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

## Direct servo and motor test

Stop `main.py`, lift both drive wheels, and run:

```bash
python3 hardware_test.py
```

The script first requires typing `LIFTED`, then tests steering directly through
Pi GPIO12 (physical pin 32). For motor tests it also verifies that the Pico
answers `PING` and clears the e-stop latch left by a normal `main.py` shutdown.
It moves steering center/left/center/right/center and runs both motors forward
and reverse at 15% requested throttle for one second each.

To isolate one output or select another UART:

```bash
python3 hardware_test.py --servo-only
python3 hardware_test.py --motor-only
python3 hardware_test.py --port /dev/serial0
```

Motor throttle is deliberately limited to 30% in this diagnostic. The script
always sends zero motor output and centers the steering before closing UART.

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
10. While steering, press and release the analog triggers and confirm they do
    not change the wheel angle.
11. Repeat the steering sweep at low motor throttle, then inspect
    `basic_rpi_car.log` for matching `steering_input`, `steering_apply`, and
    `steering_health` pulse values.
