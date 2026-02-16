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
  // Autonomous driving telemetry
  autonomous_mode?: boolean;
  autonomous_state?: string;
  autonomous_target_speed?: number;
  sonar_distance?: number;
  sonar_enabled?: boolean;
  mpu6050_enabled?: boolean;
  rear_sonar_enabled?: boolean;
  // MPU6050 Gyro telemetry
  gyro_z?: number;
  pid_correction?: number;
  gyro_available?: boolean;
  gyro_calibrated?: boolean;
  // Slalom yaw-tracking telemetry
  target_yaw?: number;
  current_heading?: number;
  slalom_sign?: number;
  // Sensor health
  sensor_status?: Record<string, string>;
  service_light_active?: boolean;
  // Camera / Vision telemetry
  camera_enabled?: boolean;
  vision_active?: boolean;
  camera_obstacle_distance?: number;
  camera_detections_count?: number;
  camera_in_path_count?: number;
  camera_closest_object?: string;
  camera_closest_confidence?: number;
  vision_fps?: number;
  // Camera configuration (from telemetry for HUD badge)
  camera_resolution?: string;
  camera_jpeg_quality?: number;
  camera_framerate?: number;
  // AI Narration telemetry
  narration_enabled?: boolean;
  narration_speaking?: boolean;
  // Camera actual FPS
  camera_actual_fps?: number;
  // Speed Encoder telemetry
  speed_mpm?: number;
  encoder_available?: boolean;
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

    // Register narration event listeners at connect time so they persist
    socket.on('narration_key_result', (data: NarrationKeyResult) => {
      console.log(`[Socket] 🎙️ Narration key result:`, data);
      if (narrationKeyResultCallback) narrationKeyResultCallback(data);
    });

    socket.on('narration_toggle_response', (data: { status: string; enabled: boolean; message?: string }) => {
      console.log(`[Socket] 🎙️ Narration toggle response:`, data);
      if (narrationToggleResponseCallback) narrationToggleResponseCallback(data);
    });

    // Register Kokoro TTS event listeners at connect time so they persist
    socket.on('kokoro_validation_result', (data: KokoroValidationResult) => {
      console.log(`[Socket] 🎤 Kokoro validation result:`, data);
      if (kokoroValidationResultCallback) kokoroValidationResultCallback(data);
    });

    socket.on('kokoro_config_response', (data: { status: string; message?: string }) => {
      console.log(`[Socket] 🎤 Kokoro config response:`, data);
      if (kokoroConfigResponseCallback) kokoroConfigResponseCallback(data);
    });
  });
}

// Tuning sync callback
let tuningSyncCallback: ((data: { tuning: Record<string, number>; defaults: Record<string, number> }) => void) | null = null;

/**
 * Subscribe to tuning sync events from the backend.
 * Called automatically on connect and in response to tuning_request.
 */
export function onTuningSync(callback: (data: { tuning: Record<string, number>; defaults: Record<string, number> }) => void): void {
  tuningSyncCallback = callback;
  if (socket) {
    socket.off('tuning_sync');  // avoid duplicate listeners
    socket.on('tuning_sync', (data) => {
      console.log(`[Socket] ⚙️ Tuning sync received from backend`);
      if (tuningSyncCallback) tuningSyncCallback(data);
    });
  }
}

// Camera specs callback
export interface CameraSpecs {
  model: string;
  max_resolution: [number, number];
  supported_modes: string[];
}

let cameraSpecsSyncCallback: ((data: CameraSpecs) => void) | null = null;

/**
 * Subscribe to camera specs sync events from the backend.
 * Camera specs contain model, max resolution, and supported resolutions.
 */
export function onCameraSpecsSync(callback: (data: CameraSpecs) => void): void {
  cameraSpecsSyncCallback = callback;
  if (socket) {
    socket.off('camera_specs_sync');  // avoid duplicate listeners
    socket.on('camera_specs_sync', (data: CameraSpecs) => {
      console.log(`[Socket] 📷 Camera specs received from backend:`, data);
      if (cameraSpecsSyncCallback) cameraSpecsSyncCallback(data);
    });
  }
}

// Camera config response callback
let cameraConfigResponseCallback: ((data: { status: string; current_config: { resolution: string; jpeg_quality: number; framerate: number } }) => void) | null = null;

/**
 * Subscribe to camera config response events from the backend.
 * Called after camera_config_update is processed.
 */
export function onCameraConfigResponse(callback: (data: { status: string; current_config: { resolution: string; jpeg_quality: number; framerate: number } }) => void): void {
  cameraConfigResponseCallback = callback;
  if (socket) {
    socket.off('camera_config_response');  // avoid duplicate listeners
    socket.on('camera_config_response', (data) => {
      console.log(`[Socket] 📷 Camera config response:`, data);
      if (cameraConfigResponseCallback) cameraConfigResponseCallback(data);
    });
  }
}

// ==========================================
// 📡 SENSOR CONFIG SYNC
// ==========================================

export interface SensorConfig {
  ir_enabled: boolean;
  sonar_enabled: boolean;
  mpu6050_enabled: boolean;
  rear_sonar_enabled: boolean;
}

let sensorConfigSyncCallback: ((data: SensorConfig) => void) | null = null;

/**
 * Subscribe to sensor config sync events from the backend.
 * Called automatically on connect to provide persisted sensor toggle states.
 */
export function onSensorConfigSync(callback: (data: SensorConfig) => void): void {
  sensorConfigSyncCallback = callback;
  if (socket) {
    socket.off('sensor_config_sync');  // avoid duplicate listeners
    socket.on('sensor_config_sync', (data: SensorConfig) => {
      console.log(`[Socket] 📡 Sensor config sync received from backend:`, data);
      if (sensorConfigSyncCallback) sensorConfigSyncCallback(data);
    });
  }
}

// ==========================================
// 🎙️ NARRATION SOCKET EVENTS
// ==========================================

export interface NarrationConfig {
  provider: string;
  api_key_set: boolean;
  api_key_masked: string;
  model: string;
  interval: number;
  enabled: boolean;
  models?: NarrationModel[];
  kokoro_enabled?: boolean;
  kokoro_ip?: string;
  kokoro_voice?: string;
  kokoro_voices?: string[];
}

export interface NarrationModel {
  name: string;
  display_name: string;
}

export interface NarrationKeyResult {
  valid: boolean;
  models: NarrationModel[];
  error: string;
}

let narrationConfigSyncCallback: ((data: NarrationConfig) => void) | null = null;
let narrationKeyResultCallback: ((data: NarrationKeyResult) => void) | null = null;
let narrationTextCallback: ((data: { text: string; timestamp: number }) => void) | null = null;
let narrationToggleResponseCallback: ((data: { status: string; enabled: boolean; message?: string }) => void) | null = null;

/**
 * Subscribe to narration config sync events (sent on connect)
 */
export function onNarrationConfigSync(callback: (data: NarrationConfig) => void): void {
  narrationConfigSyncCallback = callback;
  if (socket) {
    socket.off('narration_config_sync');
    socket.on('narration_config_sync', (data: NarrationConfig) => {
      console.log(`[Socket] 🎙️ Narration config sync:`, data);
      if (narrationConfigSyncCallback) narrationConfigSyncCallback(data);
    });
  }
}

/**
 * Subscribe to API key validation results
 */
export function onNarrationKeyResult(callback: (data: NarrationKeyResult) => void): void {
  narrationKeyResultCallback = callback;
  // Don't re-register the socket listener here — it's registered once at connect time
}

/**
 * Subscribe to narration text events (AI description of camera feed)
 */
export function onNarrationText(callback: (data: { text: string; timestamp: number }) => void): void {
  narrationTextCallback = callback;
  if (socket) {
    socket.off('narration_text');
    socket.on('narration_text', (data: { text: string; timestamp: number }) => {
      console.log(`[Socket] 🎙️ Narration text:`, data.text);
      if (narrationTextCallback) narrationTextCallback(data);
    });
  }
}

/**
 * Subscribe to narration toggle response events
 */
export function onNarrationToggleResponse(callback: (data: { status: string; enabled: boolean; message?: string }) => void): void {
  narrationToggleResponseCallback = callback;
  // Don't re-register the socket listener here — it's registered once at connect time
}

/**
 * Validate an AI provider API key
 */
export function emitNarrationValidateKey(provider: string, apiKey: string): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎙️ NARRATION: Validating ${provider} API key`);
    socket.emit('narration_validate_key', { provider, api_key: apiKey });
  }
}

/**
 * Update narration configuration (model, interval, etc.)
 */
export function emitNarrationConfigUpdate(config: { model?: string; interval?: number; provider?: string }): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎙️ NARRATION CONFIG UPDATE:`, config);
    socket.emit('narration_config_update', config);
  }
}

/**
 * Toggle AI narration on/off
 */
export function emitNarrationToggle(enabled: boolean): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎙️ NARRATION: ${enabled ? 'ENABLE' : 'DISABLE'}`);
    socket.emit('narration_toggle', { enabled });
  }
}

/**
 * Notify server that TTS playback has finished
 */
export function emitNarrationSpeakingDone(): void {
  if (socket && socket.connected) {
    socket.emit('narration_speaking_done', {});
  }
}

/**
 * Clear the stored API key on the backend
 */
export function emitNarrationKeyClear(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎙️ NARRATION: Clear API key`);
    socket.emit('narration_key_clear', {});
  }
}

// ==========================================
// 🎤 KOKORO TTS SOCKET EVENTS
// ==========================================

export interface KokoroValidationResult {
  valid: boolean;
  voices: string[];
  error: string | null;
}

let kokoroValidationResultCallback: ((data: KokoroValidationResult) => void) | null = null;
let kokoroConfigResponseCallback: ((data: { status: string; message?: string }) => void) | null = null;

/**
 * Subscribe to Kokoro API validation results.
 * The socket listener is registered at connect time in connectToServer().
 * This function just sets the callback.
 */
export function onKokoroValidationResult(callback: (data: KokoroValidationResult) => void): void {
  kokoroValidationResultCallback = callback;
}

/**
 * Subscribe to Kokoro config response events.
 * The socket listener is registered at connect time in connectToServer().
 * This function just sets the callback.
 */
export function onKokoroConfigResponse(callback: (data: { status: string; message?: string }) => void): void {
  kokoroConfigResponseCallback = callback;
}

/**
 * Validate Kokoro TTS API connection and fetch available voices
 */
export function emitKokoroValidateApi(ip: string): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎤 KOKORO: Validating API at ${ip}`);
    socket.emit('kokoro_validate_api', { ip });
  }
}

/**
 * Update Kokoro TTS configuration (enable/disable, IP, voice)
 */
export function emitKokoroConfigUpdate(config: { kokoro_enabled?: boolean; kokoro_ip?: string; kokoro_voice?: string }): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎤 KOKORO CONFIG UPDATE:`, config);
    socket.emit('kokoro_config_update', config);
  }
}

/**
 * Explicitly request current tuning from the backend
 */
export function requestTuning(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] ⚙️ Requesting current tuning from backend`);
    socket.emit('tuning_request', {});
  }
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
    socket.on('telemetry_update', (data: Record<string, unknown>) => {
      if (data.heartbeat_active !== undefined) {
        callback(data.heartbeat_active as boolean);
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
 * Toggle Rear Sonar sensor control
 */
export function emitRearSonarToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 REAR SONAR: TOGGLED`);
    socket.emit('rear_sonar_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit rear sonar toggle - socket not connected`);
  }
}

/**
 * Toggle MPU6050 gyroscope sensor control
 */
export function emitMPU6050Toggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 MPU6050: TOGGLED`);
    socket.emit('mpu6050_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit MPU6050 toggle - socket not connected`);
  }
}

/**
 * Toggle Camera (enables/disables camera and all vision-related functions)
 */
export function emitCameraToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 CAMERA: TOGGLED`);
    socket.emit('camera_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit camera toggle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Toggle Vision/Object Detection
 */
export function emitVisionToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 VISION: TOGGLED`);
    socket.emit('vision_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit vision toggle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Update camera configuration (resolution, quality, framerate)
 */
export function emitCameraConfigUpdate(config: {
  resolution?: string;
  jpeg_quality?: number;
  framerate?: number;
}): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 CAMERA CONFIG UPDATE:`, config);
    socket.emit('camera_config_update', config);
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit camera config update - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Toggle Autopilot control
 */
export function emitAutopilotToggle(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 AUTOPILOT: TOGGLED`);
    socket.emit('autonomous_toggle', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit autopilot toggle - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Explicitly enable Autopilot (idempotent — safe to call if already enabled)
 */
export function emitAutopilotEnable(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 AUTOPILOT: ENABLE`);
    socket.emit('autonomous_enable', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit autopilot enable - socket not connected`);
  }
}

/**
 * Explicitly disable Autopilot (idempotent — safe to call if already disabled)
 */
export function emitAutopilotDisable(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 AUTOPILOT: DISABLE`);
    socket.emit('autonomous_disable', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit autopilot disable - socket not connected`);
  }
}

/**
 * Emit tuning constants update to the backend autopilot
 * @param tuning - Object containing all tuning constants
 */
export function emitTuningUpdate(tuning: Record<string, number>): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] ⚙️ TUNING UPDATE: Sending ${Object.keys(tuning).length} constants`);
    socket.emit('tuning_update', { tuning });
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit tuning update - socket not connected`);
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
  emitRearSonarToggle,
  emitMPU6050Toggle,
  emitCameraToggle,
  emitVisionToggle,
  emitCameraConfigUpdate,
  emitAutopilotToggle,
  emitAutopilotEnable,
  emitAutopilotDisable,
  emitTuningUpdate,
  onTuningSync,
  onCameraSpecsSync,
  onCameraConfigResponse,
  requestTuning,
  // Narration
  onNarrationConfigSync,
  onNarrationKeyResult,
  onNarrationText,
  onNarrationToggleResponse,
  emitNarrationValidateKey,
  emitNarrationConfigUpdate,
  emitNarrationToggle,
  emitNarrationSpeakingDone,
};
