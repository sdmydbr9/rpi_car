# Mobile App Implementation Guide

## Overview

This repository now includes a React Native mobile application located in the `./app` directory, supporting both Android and iOS platforms. The mobile app provides remote control capabilities for the RPi Car.

## What Was Implemented

### 1. Directory Structure
- **./app/android**: Native Android project with full build configuration
- **./app/ios**: Native iOS project with Xcode configuration
- **./app/src**: TypeScript source code for the mobile app

### 2. Core Components

#### Socket Client (`app/src/lib/socketClient.ts`)
A TypeScript library that handles WebSocket communication with the RPi Car backend using Socket.IO. Features include:
- Connection management
- Telemetry data reception
- Control commands (throttle, brake, steering, gear changes)
- Emergency stop functionality
- Autopilot control

#### Control Screen (`app/src/screens/ControlScreen.tsx`)
The main UI for controlling the car with:
- Server connection interface
- Real-time telemetry display (speed, RPM, temperature, etc.)
- Steering controls (left, center, right)
- Throttle and brake buttons
- Gear shifter (R, N, 1, 2, 3)
- Emergency stop button
- Dark theme optimized for outdoor use

### 3. Technical Stack
- **React Native**: Cross-platform mobile framework
- **Expo**: Development platform and build tooling
- **TypeScript**: Type-safe development
- **Socket.IO Client**: Real-time communication
- **React Navigation**: Screen navigation

### 4. Configuration
- **Landscape orientation**: Optimized for control interface
- **Dark theme**: Better visibility outdoors
- **Package identifiers**: `com.rpicar.control`
- **Default server**: `192.168.4.1:5000`

## Key Differences from Web App

### Web App (Original)
- Uses Vite + React for web browsers
- Relies on browser-specific APIs (WebRTC, DOM)
- Complex UI with Radix UI components
- Tailwind CSS for styling

### Mobile App (New)
- Uses React Native for native mobile platforms
- Native mobile APIs and components
- Simplified UI with React Native components
- StyleSheet API for styling
- Expo for easier development and deployment

## How to Use

### Development
```bash
cd app
npm install
npm start
```

### Run on Device
```bash
# Android
npm run android

# iOS (macOS only)
npm run ios
```

### Build for Production
```bash
# Android APK
npx expo build:android

# iOS App
npx expo build:ios
```

## Future Enhancements

Potential features to add:
1. **Camera feed integration**: Display live camera feed from the car
2. **Advanced telemetry**: More detailed sensor data visualization
3. **Route recording**: GPS tracking and playback
4. **Autopilot tuning**: UI for adjusting autopilot parameters
5. **Voice commands**: Hands-free control
6. **Multiple car support**: Switch between different cars
7. **Offline mode**: Queue commands when disconnected

## Project Structure Explanation

```
rpi_car/
├── app/                          # Mobile application (NEW)
│   ├── android/                  # Android native project
│   │   ├── app/                  # Main Android app module
│   │   │   ├── src/main/         # Android source files
│   │   │   │   ├── AndroidManifest.xml
│   │   │   │   ├── java/         # Kotlin/Java source
│   │   │   │   └── res/          # Android resources
│   │   │   └── build.gradle      # App-level Gradle config
│   │   ├── gradle/               # Gradle wrapper
│   │   ├── build.gradle          # Root-level Gradle config
│   │   └── settings.gradle       # Gradle settings
│   │
│   ├── ios/                      # iOS native project
│   │   ├── app/                  # Main iOS app
│   │   │   ├── AppDelegate.swift # iOS app delegate
│   │   │   ├── Info.plist        # iOS app configuration
│   │   │   └── Images.xcassets/  # iOS image assets
│   │   ├── app.xcodeproj/        # Xcode project files
│   │   └── Podfile               # CocoaPods dependencies
│   │
│   ├── src/                      # React Native source code
│   │   ├── screens/              # Screen components
│   │   │   └── ControlScreen.tsx # Main control interface
│   │   ├── lib/                  # Utilities and libraries
│   │   │   └── socketClient.ts   # Socket.IO client
│   │   ├── components/           # Reusable components
│   │   └── hooks/                # Custom React hooks
│   │
│   ├── App.tsx                   # Root app component
│   ├── app.json                  # Expo configuration
│   ├── package.json              # Dependencies
│   └── README.md                 # Mobile app documentation
│
├── src/                          # Web app source (EXISTING)
│   ├── pages/                    # Web pages
│   ├── components/               # Web components
│   └── lib/                      # Web utilities
│
├── main.py                       # Python backend (EXISTING)
├── package.json                  # Web app dependencies
└── README.md                     # Main project documentation
```

## Architecture

### Communication Flow
```
Mobile App (React Native)
    ↓ Socket.IO over WiFi
RPi Car Backend (Python/Flask)
    ↓ GPIO/Serial
RPi Car Hardware (Motors, Sensors)
```

### Data Flow
1. **User Input**: User interacts with mobile UI
2. **Command Emission**: Socket client sends commands to backend
3. **Backend Processing**: Python backend processes commands
4. **Hardware Control**: Backend controls motors/sensors via GPIO
5. **Telemetry Collection**: Backend reads sensor data
6. **Telemetry Broadcast**: Backend sends updates to mobile app
7. **UI Update**: Mobile app displays updated telemetry

## Notes

- The mobile app and web app can both connect to the same backend simultaneously
- The backend at `192.168.4.1:5000` must be running for the app to function
- The mobile device must be on the same network as the RPi Car
- For development, you can use `localhost` if running the backend on your development machine

## Troubleshooting

### Cannot connect to server
- Verify the RPi Car backend is running
- Check that the IP address is correct (default: 192.168.4.1)
- Ensure your mobile device is on the same WiFi network

### Build errors
- Run `npm install` to ensure all dependencies are installed
- Clear Expo cache: `expo start -c`
- Rebuild native projects: `npx expo prebuild --clean`

### TypeScript errors
- Check that all dependencies are installed
- Run `npx tsc --noEmit` to see detailed type errors

## Contributing

When adding new features to the mobile app:
1. Keep components in `src/components/`
2. Keep screens in `src/screens/`
3. Keep utilities in `src/lib/`
4. Update this documentation
5. Test on both iOS and Android if possible
