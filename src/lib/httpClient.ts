import { getAutoAccelState, getEmergencyStopState } from '../hooks/useAutoAcceleration';

// HTTP client configuration
let serverUrl: string = '';
let telemetryIntervalId: number | null = null;

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
}

/**
 * Initialize connection to the HTTP backend
 * @param serverIp - The IP address of the Raspberry Pi (e.g., "192.168.4.1" or "localhost")
 * @param port - The port number (default 5000)
 * @returns Promise that resolves when connection is verified
 */
export function connectToServer(serverIp: string, port: number = 5000): Promise<void> {
  return new Promise((resolve, reject) => {
    serverUrl = `http://${serverIp}:${port}`;
    console.log(`[HTTP Client] 🔌 Attempting to connect to: ${serverUrl}`);

    // Test connection with a simple health check
    fetch(`${serverUrl}/api/state`)
      .then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        console.log(`[HTTP Client] ✅ Successfully connected to RC Car backend at ${serverUrl}`);
        
        // Start telemetry polling
        startTelemetryPolling();
        resolve();
      })
      .catch((error) => {
        console.error(`[HTTP Client] ❌ Connection error to ${serverUrl}:`, error);
        reject(error);
      });
  });
}

/**
 * Disconnect from the backend
 */
export function disconnectFromServer(): void {
  console.log('[HTTP Client] 🔓 Disconnecting from RC Car backend');
  stopTelemetryPolling();
  serverUrl = '';
  telemetryCallback = null;
}

/**
 * Check if connected to the backend
 */
export function isConnected(): boolean {
  return serverUrl.length > 0;
}

/**
 * Subscribe to telemetry updates
 */
export function onTelemetry(callback: (data: TelemetryData) => void): void {
  telemetryCallback = callback;
}

/**
 * Start polling telemetry data at 20Hz (50ms intervals)
 */
function startTelemetryPolling(): void {
  if (telemetryIntervalId !== null) {
    clearInterval(telemetryIntervalId);
  }

  telemetryIntervalId = window.setInterval(() => {
    if (!isConnected() || !telemetryCallback) return;

    fetch(`${serverUrl}/api/telemetry`)
      .then((response) => response.json())
      .then((data: TelemetryData) => {
        if (telemetryCallback) {
          telemetryCallback(data);
        }
      })
      .catch((error) => {
        console.warn('[HTTP Client] ⚠️ Failed to fetch telemetry:', error);
      });
  }, 50); // 20Hz polling
}

/**
 * Stop polling telemetry data
 */
function stopTelemetryPolling(): void {
  if (telemetryIntervalId !== null) {
    clearInterval(telemetryIntervalId);
    telemetryIntervalId = null;
  }
}

/**
 * Helper function to make API requests
 */
async function apiRequest(endpoint: string, method: string = 'POST', data?: any): Promise<any> {
  if (!isConnected()) {
    console.warn(`[HTTP Client] ⚠️ Cannot make request - not connected to server`, { endpoint });
    return { status: 'error', message: 'Not connected' };
  }

  try {
    const options: RequestInit = {
      method,
      headers: { 'Content-Type': 'application/json' },
    };

    if (data) {
      options.body = JSON.stringify(data);
    }

    const response = await fetch(`${serverUrl}${endpoint}`, options);
    return await response.json();
  } catch (error) {
    console.error(`[HTTP Client] ❌ API request error:`, error);
    return { status: 'error', message: error instanceof Error ? error.message : 'Unknown error' };
  }
}

/**
 * Emit throttle control
 * @param pressed - Whether throttle is pressed (true = gas, false = release)
 */
export function emitThrottle(pressed: boolean): void {
  console.log(`[UI Control] 🎮 THROTTLE: ${pressed ? '🚀 GAS PRESSED' : '❌ RELEASED'}`);
  apiRequest('/api/control/throttle', 'POST', { value: pressed });
}

/**
 * Emit brake control
 * @param pressed - Whether brake is pressed
 */
export function emitBrake(pressed: boolean): void {
  console.log(`[UI Control] 🎮 BRAKE: ${pressed ? '🛑 ENGAGED' : '✋ RELEASED'}`);
  apiRequest('/api/control/brake', 'POST', { value: pressed });
}

/**
 * Emit steering control
 * @param angle - Steering angle from -90 (left) to +90 (right)
 */
export function emitSteering(angle: number): void {
  // Clamp angle between -90 and 90
  const clampedAngle = Math.max(-90, Math.min(90, angle));
  const direction = clampedAngle < 0 ? '⬅️ LEFT' : clampedAngle > 0 ? '➡️ RIGHT' : '⬆️ CENTER';
  console.log(`[UI Control] 🎮 STEERING: ${direction} (${clampedAngle}°)`);
  apiRequest('/api/control/steering', 'POST', { angle: clampedAngle });
}

/**
 * Emit gear change
 * @param gear - The gear to select: R, N, 1, 2, 3, S
 */
export function emitGearChange(gear: string): void {
  const gearLabel = { 'R': '🔙 REVERSE', 'N': '⏸️ NEUTRAL', '1': '1️⃣ 1st', '2': '2️⃣ 2nd', '3': '3️⃣ 3rd', 'S': '⚡ SPORT' }[gear.toUpperCase()] || gear;
  console.log(`[UI Control] 🎮 GEAR CHANGE: ${gearLabel}`);
  apiRequest('/api/control/gear', 'POST', { gear: gear.toUpperCase() });
}

/**
 * Emit emergency stop
 */
export function emitEmergencyStop(): void {
  console.log(`[UI Control] 🚨 EMERGENCY STOP ACTIVATED`);
  apiRequest('/api/control/emergency_stop', 'POST', {});
}

/**
 * Enable auto-acceleration mode
 */
export function emitAutoAccelEnable(): void {
  console.log(`[UI Control] 🎮 AUTO-ACCEL: ENABLED (auto-throttle mode)`);
  apiRequest('/api/control/auto_accel_enable', 'POST', {});
}

/**
 * Disable auto-acceleration mode
 */
export function emitAutoAccelDisable(): void {
  console.log(`[UI Control] 🎮 AUTO-ACCEL: DISABLED`);
  apiRequest('/api/control/auto_accel_disable', 'POST', {});
}

/**
 * Toggle IR sensor control
 */
export function emitIRToggle(): void {
  console.log(`[UI Control] 🎮 IR SENSORS: TOGGLED`);
  apiRequest('/api/control/ir_toggle', 'POST', {});
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
  emitAutoAccelEnable,
  emitAutoAccelDisable,
  emitIRToggle,
};
