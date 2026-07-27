# Mobile Manual-Control App

The Expo app in `app/` uses the same direct-control protocol as the browser.
It contains no camera, telemetry, autonomous, or sensor stream.

## Install and run after transfer

```bash
cd /home/pi/rpi_car/app
npm install
npm run typecheck
npm run android
```

For iOS, use `npm run ios` on a supported macOS system.
The root `npm test` command also exercises the mobile spring-return and active
control refresh policy.

The default Pi address is `192.168.4.1`. Enter another Pi address before
connecting when using normal Wi-Fi.

## Safety behavior

- Take Control before arming.
- A connected Pi gamepad blocks mobile drive commands.
- Throttle and steering return to zero on release.
- Backgrounding the app sends zero and disarms.
- Moving control state is refreshed every 100 ms.
- E-stop can be engaged without ownership; reset requires priority ownership.
- Battery is read only on request while disarmed.
- Wi-Fi/hotspot switching is available with the car disarmed and disconnects
  the current session.

Complete `TRANSFER_CHECKLIST.md` before the first ground drive.
