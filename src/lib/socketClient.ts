import { io, Socket } from 'socket.io-client';

// Socket instance
let socket: Socket | null = null;

// Telemetry callback
let telemetryCallback: ((data: TelemetryData) => void) | null = null;

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
}

/**
 * Initialize WebSocket connection to the backend
 * @param serverIp - The IP address of the Raspberry Pi (e.g., "192.168.4.1" or "localhost")
 * @param port - The port number (default 5000)
 * @returns Promise that resolves when connected
 */
export function connectToServer(serverIp: string, port: number = 5000): Promise<void> {
  return new Promise((resolve, reject) => {
    const url = `http://${serverIp}:${port}`;
    console.log(`[Socket] 🔌 Attempting to connect to: ${url}`);

    socket = io(url, {
      reconnection: true,
      reconnectionDelay: 1000,
      reconnectionDelayMax: 5000,
      reconnectionAttempts: 5,
      transports: ['websocket', 'polling'],
    });

    socket.on('connect', () => {
      console.log(`[Socket] ✅ Successfully connected to RC Car backend at ${url}`);
      console.log(`[Socket] 📡 Socket ID: ${socket?.id}`);
      resolve();
    });

    socket.on('connection_response', (data) => {
      console.log(`[Socket] 💬 Server response:`, data);
    });

    socket.on('telemetry_update', (data: TelemetryData) => {
      if (telemetryCallback) {
        telemetryCallback(data);
      }
    });

    socket.on('connect_error', (error) => {
      console.error(`[Socket] ❌ Connection error to ${url}:`, error);
      reject(error);
    });

    socket.on('disconnect', () => {
      console.log(`[Socket] ❌ Disconnected from RC Car backend`);
    });

    socket.on('heartbeat_ping', () => {
      console.log(`[Socket] 💗 Heartbeat ping received from server, sending heartbeat_pong...`);
      socket?.emit('heartbeat_pong', {});
    });
  });
}

/**
 * Disconnect from the backend
 */
export function disconnectFromServer(): void {
  if (socket) {
    socket.disconnect();
    socket = null;
    telemetryCallback = null;
  }
}

/**
 * Check if connected to the backend
 */
export function isConnected(): boolean {
  return socket?.connected || false;
}

/**
 * Get heartbeat status from server (True = heartbeat active, False = lost)
 */
export function onHeartbeatStatus(callback: (active: boolean) => void): void {
  if (socket) {
    socket.on('telemetry_update', (data: any) => {
      if (data.heartbeat_active !== undefined) {
        callback(data.heartbeat_active);
      }
    });
  }
}


/**
 * Subscribe to telemetry updates
 */
export function onTelemetry(callback: (data: TelemetryData) => void): void {
  telemetryCallback = callback;
}

/**
 * Emit throttle control
 * @param pressed - Whether throttle is pressed (true = gas, false = release)
 */
export function emitThrottle(pressed: boolean): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 THROTTLE: ${pressed ? '🚀 GAS PRESSED' : '❌ RELEASED'}`);
    socket.emit('throttle', { value: pressed });
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit throttle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit brake control
 * @param pressed - Whether brake is pressed
 */
export function emitBrake(pressed: boolean): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 BRAKE: ${pressed ? '🛑 ENGAGED' : '✋ RELEASED'}`);
    socket.emit('brake', { value: pressed });
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit brake - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit steering control
 * @param angle - Steering angle from -90 (left) to +90 (right)
 */
export function emitSteering(angle: number): void {
  if (socket && socket.connected) {
    // Clamp angle between -90 and 90
    const clampedAngle = Math.max(-90, Math.min(90, angle));
    const direction = clampedAngle < 0 ? '⬅️ LEFT' : clampedAngle > 0 ? '➡️ RIGHT' : '⬆️ CENTER';
    console.log(`[UI Control] 🎮 STEERING: ${direction} (${clampedAngle}°)`);
    socket.emit('steering', { angle: clampedAngle });
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit steering - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit gear change
 * @param gear - The gear to select: R, N, 1, 2, 3, S
 */
export function emitGearChange(gear: string): void {
  if (socket && socket.connected) {
    const gearLabel = { 'R': '🔙 REVERSE', 'N': '⏸️ NEUTRAL', '1': '1️⃣ 1st', '2': '2️⃣ 2nd', '3': '3️⃣ 3rd', 'S': '⚡ SPORT' }[gear.toUpperCase()] || gear;
    console.log(`[UI Control] 🎮 GEAR CHANGE: ${gearLabel}`);
    socket.emit('gear_change', { gear: gear.toUpperCase() });
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit gear change - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit emergency stop
 */
export function emitEmergencyStop(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🚨 EMERGENCY STOP ACTIVATED`);
    socket.emit('emergency_stop', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit emergency stop - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit emergency stop release
 */
export function emitEmergencyStopRelease(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🚨 EMERGENCY STOP RELEASED`);
    socket.emit('emergency_stop_release', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit emergency stop release - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Enable auto-acceleration mode
 */
export function emitAutoAccelEnable(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 AUTO-ACCEL: ENABLED (auto-throttle mode)`);
    socket.emit('auto_accel_enable', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit auto accel enable - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Disable auto-acceleration mode
 */
export function emitAutoAccelDisable(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 AUTO-ACCEL: DISABLED`);
    socket.emit('auto_accel_disable', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit auto accel disable - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Toggle IR sensor control
 */
export function emitIRToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 IR SENSORS: TOGGLED`);
    socket.emit('ir_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit IR toggle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Toggle SONAR sensor control
 */
export function emitSonarToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 SONAR: TOGGLED`);
    socket.emit('sonar_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit sonar toggle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Toggle Autopilot control
 */
export function emitAutopilotToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 AUTOPILOT: TOGGLED`);
    socket.emit('autopilot_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit autopilot toggle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

export default {
  connectToServer,
  disconnectFromServer,
  isConnected,
  onTelemetry,
  emitThrottle,
  emitBrake,
  emitSteering,
  emitGearChange,
  emitEmergencyStop,
  emitEmergencyStopRelease,
  emitAutoAccelEnable,
  emitAutoAccelDisable,
  emitIRToggle,
  emitSonarToggle,
  emitAutopilotToggle,
};
