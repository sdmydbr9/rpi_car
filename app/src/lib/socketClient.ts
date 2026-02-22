import { io, Socket } from 'socket.io-client';

// Socket instance
let socket: Socket | null = null;

// Telemetry callback
let telemetryCallback: ((data: TelemetryData) => void) | null = null;

// Connection state callback - fired when socket connects/disconnects
let connectionStateCallback: ((isConnected: boolean) => void) | null = null;

export interface TelemetryData {
  rpm: number;
  speed: number;
  current_pwm: number;
  gear: string;
  steer_angle: number;
  direction: string;
  turning: string;
  left_obstacle: boolean;
  right_obstacle: boolean;
  gas_pressed: boolean;
  brake_pressed: boolean;
  ir_enabled: boolean;
  heartbeat_active?: boolean;
  emergency_brake_active?: boolean;
  temperature?: number;
  cpu_clock?: number;
  gpu_clock?: number;
  autonomous_mode?: boolean;
  autonomous_state?: string;
  autonomous_target_speed?: number;
  sonar_distance?: number;
  sonar_enabled?: boolean;
  camera_enabled?: boolean;
  vision_active?: boolean;
}

export function connectToServer(serverIp: string, port: number = 5000): Promise<void> {
  return new Promise((resolve, reject) => {
    const url = `http://${serverIp}:${port}`;
    console.log(`[Socket] Attempting to connect to: ${url}`);

    socket = io(url, {
      reconnection: true,
      reconnectionDelay: 1000,
      reconnectionDelayMax: 5000,
      reconnectionAttempts: 5,
      transports: ['websocket', 'polling'],
    });

    socket.on('connect', () => {
      console.log(`[Socket] Successfully connected to RC Car backend at ${url}`);
      if (connectionStateCallback) {
        connectionStateCallback(true);
      }
      resolve();
    });

    socket.on('telemetry_update', (data: TelemetryData) => {
      if (telemetryCallback) {
        telemetryCallback(data);
      }
    });

    socket.on('connect_error', (error) => {
      console.error(`[Socket] Connection error to ${url}:`, error);
      reject(error);
    });

    socket.on('disconnect', () => {
      console.log(`[Socket] Disconnected from RC Car backend`);
      if (connectionStateCallback) {
        connectionStateCallback(false);
      }
    });

    socket.on('heartbeat_ping', () => {
      socket?.emit('heartbeat_pong', {});
    });
  });
}

export function disconnectFromServer(): void {
  if (socket) {
    socket.disconnect();
    socket = null;
    telemetryCallback = null;
  }
}

export function isConnected(): boolean {
  return socket?.connected || false;
}

export function onConnectionStateChange(callback: (isConnected: boolean) => void): void {
  connectionStateCallback = callback;
}

export function onTelemetry(callback: (data: TelemetryData) => void): void {
  telemetryCallback = callback;
}

export function emitThrottle(pressed: boolean): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] THROTTLE: ${pressed ? 'PRESSED' : 'RELEASED'}`);
    socket.emit('throttle', { value: pressed });
  }
}

export function emitBrake(pressed: boolean): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] BRAKE: ${pressed ? 'ENGAGED' : 'RELEASED'}`);
    socket.emit('brake', { value: pressed });
  }
}

export function emitSteering(angle: number): void {
  if (socket && socket.connected) {
    const clampedAngle = Math.max(-90, Math.min(90, angle));
    console.log(`[UI Control] STEERING: ${clampedAngle}°`);
    socket.emit('steering', { angle: clampedAngle });
  }
}

export function emitGearChange(gear: string): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] GEAR CHANGE: ${gear}`);
    socket.emit('gear_change', { gear: gear.toUpperCase() });
  }
}

export function emitEmergencyStop(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] EMERGENCY STOP ACTIVATED`);
    socket.emit('emergency_stop', {});
  }
}

export function emitEmergencyStopRelease(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] EMERGENCY STOP RELEASED`);
    socket.emit('emergency_stop_release', {});
  }
}

export function emitIRToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] IR SENSORS: TOGGLED`);
    socket.emit('ir_toggle', {});
  }
}

export function emitSonarToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] SONAR: TOGGLED`);
    socket.emit('sonar_toggle', {});
  }
}

export function emitAutopilotToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] AUTOPILOT: TOGGLED`);
    socket.emit('autonomous_toggle', {});
  }
}
