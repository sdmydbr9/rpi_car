# Transfer and First-Drive Checklist

## Before powering motors

- [ ] Copy `scripts/firmware/pico_manual_controller.py` to the Pico as `main.py`.
- [ ] Confirm Pi↔Pico ground, TX/RX crossover, and 115200 baud configuration.
- [ ] Confirm the L298N and steering pins match `README.md`.
- [ ] Confirm the ADS1115 is at `0x48` and the battery divider is connected to A0.
- [ ] Set `RC_BATTERY_DIVIDER_RATIO` if the divider is not 5:1.
- [ ] Install Python and Node dependencies and build the web client.
- [ ] Add the service user to `dialout` and `input`.

## Stationary validation

- [ ] Lift all drive wheels clear of the surface.
- [ ] Run `python -m unittest discover -s tests -v`.
- [ ] Run `npm test`, `npm run lint`, and `npm run build`.
- [ ] Run `cd app && npm run typecheck`.
- [ ] Run `python scripts/diagnostics/manual_control_smoke.py`.
- [ ] Confirm `PING` succeeds and battery voltage is plausible.
- [ ] Confirm steering centers and never exceeds the calibrated mechanical limits.
- [ ] Run the smoke utility with `--allow-motor-test`.
- [ ] Confirm both rear wheels rotate forward and stop after one second.

## Watchdog and control validation

- [ ] Drive at low throttle with wheels lifted, then disconnect the browser.
- [ ] Confirm motor output stops within 300 ms.
- [ ] Disconnect Pi UART TX while driving at low throttle.
- [ ] Confirm the Pico watchdog stops motor output within 300 ms.
- [ ] Attempt a direct F→R change and confirm output goes to zero first.
- [ ] Engage e-stop from a non-owner client and confirm it stops immediately.
- [ ] Confirm e-stop cannot reset while armed or while the gamepad has priority.
- [ ] Connect a gamepad while a remote owns control and confirm remote drive stops.
- [ ] Disconnect the gamepad and confirm the car remains disarmed.

## Data-flow validation

- [ ] Observe Pico TX while idle and confirm there are no periodic packets.
- [ ] Confirm the browser receives no `telemetry_update` events.
- [ ] Confirm battery data appears only after pressing Read Battery while disarmed.
- [ ] Confirm no camera, MediaMTX, Grafana, ROS, or AI services are started.

## Ground test

- [ ] Start with a clear, open test area and the lowest practical throttle.
- [ ] Verify forward, reverse, steering, brake, disarm, and e-stop individually.
- [ ] Increase throttle only after all safety actions have been verified.
