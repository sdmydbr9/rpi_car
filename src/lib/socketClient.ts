import { io, Socket } from 'socket.io-client';

// Socket instance
let socket: Socket | null = null;

// Telemetry callback
let telemetryCallback: ((data: TelemetryData) => void) | null = null;
let cameraResponseCallback: ((data: CameraResponse) => void) | null = null;

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
  engine_running?: boolean;
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
  camera_effective_stream_fps_limit?: number;
  camera_effective_jpeg_quality?: number;
  camera_adaptive_overloaded?: boolean;
  camera_stream_backend?: 'mediamtx_webrtc';
  camera_h264_rtsp_running?: boolean;
  camera_h264_rtsp_url?: string;
  camera_h264_rtsp_transport?: 'rtsp';
  camera_mediamtx_running?: boolean;
  camera_mediamtx_webrtc_url?: string;
  camera_mediamtx_rtsp_ingest_url?: string;
  camera_mediamtx_error?: string;
  // Speed Encoder telemetry
  speed_mpm?: number;
  encoder_available?: boolean;
  // Battery telemetry
  battery_voltage?: number;
  // Gamepad telemetry
  gamepad_connected?: boolean;
  gamepad_enabled?: boolean;
  gamepad_gear?: string;
}

export interface CameraResponse {
  status: 'ok' | 'error' | 'blocked';
  message?: string;
  camera_enabled?: boolean;
  backend?: string;
  mediamtx_running?: boolean;
  mediamtx_webrtc_url?: string;
  mediamtx_error?: string;
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
      reconnectionAttempts: 10,  // Increased for hotspot stability
      transports: ['websocket', 'polling'],
      // Hotspot mode configuration
      connectTimeout: 10000,  // 10 second timeout for initial connection
      // Polling transport settings for hotspot mode reliability
      upgrade: true,
      forceNew: false,
      // Disable secure WebSocket requirement for local hotspot
      secure: false,
    });

    // Timeout to reject if connection takes too long
    const connectionTimeout = setTimeout(() => {
      if (!socket?.connected) {
        console.error(`[Socket] ❌ Connection timeout to ${url}`);
        socket?.close();
        reject(new Error(`Connection timeout to ${url}`));
      }
    }, 15000);  // 15 second overall timeout

    socket.on('connect', () => {
      clearTimeout(connectionTimeout);
      console.log(`[Socket] ✅ Successfully connected to RC Car backend at ${url}`);
      console.log(`[Socket] 📡 Socket ID: ${socket?.id}`);
      if (connectionStateCallback) {
        connectionStateCallback(true);
      }
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

    socket.on('camera_response', (data: CameraResponse) => {
      console.log(`[Socket] 📷 Camera response:`, data);
      if (cameraResponseCallback) {
        cameraResponseCallback(data);
      }
    });

    socket.on('connect_error', (error) => {
      clearTimeout(connectionTimeout);
      console.error(`[Socket] ❌ Connection error to ${url}:`, error);
      reject(error);
    });

    socket.on('error', (error) => {
      console.error(`[Socket] ❌ Socket error:`, error);
    });

    socket.on('disconnect', () => {
      console.log(`[Socket] ❌ Disconnected from RC Car backend`);
      if (connectionStateCallback) {
        connectionStateCallback(false);
      }
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

    socket.on('narration_analyze_once_response', (data: NarrationAnalyzeOnceResponse) => {
      console.log(`[Socket] 🎙️ Narration analyze-once response:`, data);
      if (narrationAnalyzeOnceResponseCallback) narrationAnalyzeOnceResponseCallback(data);
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

    // Register ElevenLabs TTS event listeners at connect time so they persist
    socket.on('elevenlabs_validation_result', (data: ElevenLabsValidationResult) => {
      console.log(`[Socket] 🎤 ElevenLabs validation result:`, data);
      if (elevenLabsValidationResultCallback) elevenLabsValidationResultCallback(data);
    });

    // Register startup sync/result listeners at connect time so callbacks
    // still work when subscribed before socket creation.
    socket.on('startup_config_sync', (data: StartupConfig) => {
      console.log(`[Socket] ⬛ Startup config sync:`, data);
      if (startupConfigSyncCallback) startupConfigSyncCallback(data);
    });

    socket.on('startup_check_result', (data: StartupCheckResult) => {
      console.log(`[Socket] ⬛ Startup check result:`, data);
      if (startupCheckResultCallback) startupCheckResultCallback(data);
    });

    // Register gamepad event listeners at connect time so they persist across reconnects
    socket.on('gamepad_start_pressed', (data: { engine_running: boolean }) => {
      if (gamepadStartPressedCallback) gamepadStartPressedCallback(data);
    });

    socket.on('autonomous_update', (data: { autonomous_mode: boolean; source?: string }) => {
      if (autonomousUpdateCallback) autonomousUpdateCallback(data);
    });

    socket.on('gamepad_hot_start', (data: { engine_running: boolean; gamepad_enabled: boolean }) => {
      if (gamepadHotStartCallback) gamepadHotStartCallback(data);
    });

    socket.on('gamepad_sensor_toggle', (data: { sensor: string; enabled: boolean }) => {
      if (gamepadSensorToggleCallback) gamepadSensorToggleCallback(data);
    });

    socket.on('gamepad_autoaccel_update', (data: { auto_accel_enabled: boolean; source?: string }) => {
      if (gamepadAutoAccelCallback) gamepadAutoAccelCallback(data);
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

interface CameraConfigResponse {
  status: string;
  message?: string;
  current_config: {
    resolution: string;
    jpeg_quality: number;
    framerate: number;
    stream_backend?: 'mediamtx_webrtc';
  };
  mediamtx?: {
    running: boolean;
    webrtc_url: string;
    rtsp_ingest_url?: string;
    error?: string;
  };
  h264_rtsp?: {
    running: boolean;
    url: string;
    transport?: 'rtsp';
  };
}

// Camera config response callback
let cameraConfigResponseCallback: ((data: CameraConfigResponse) => void) | null = null;

/**
 * Subscribe to camera config response events from the backend.
 * Called after camera_config_update is processed.
 */
export function onCameraConfigResponse(callback: (data: CameraConfigResponse) => void): void {
  cameraConfigResponseCallback = callback;
  if (socket) {
    socket.off('camera_config_response');  // avoid duplicate listeners
    socket.on('camera_config_response', (data) => {
      console.log(`[Socket] 📷 Camera config response:`, data);
      if (cameraConfigResponseCallback) cameraConfigResponseCallback(data);
    });
  }
}

/**
 * Subscribe to camera toggle/status responses.
 */
export function onCameraResponse(callback: (data: CameraResponse) => void): void {
  cameraResponseCallback = callback;
  if (socket) {
    socket.off('camera_response');
    socket.on('camera_response', (data: CameraResponse) => {
      console.log(`[Socket] 📷 Camera response:`, data);
      if (cameraResponseCallback) cameraResponseCallback(data);
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
  elevenlabs_api_key_set?: boolean;
  elevenlabs_api_key?: string;
  elevenlabs_api_key_masked?: string;
  elevenlabs_voice_id?: string;
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

export interface NarrationAnalyzeOnceResponse {
  status: 'ok' | 'error';
  message?: string;
}

let narrationConfigSyncCallback: ((data: NarrationConfig) => void) | null = null;
let narrationKeyResultCallback: ((data: NarrationKeyResult) => void) | null = null;
let narrationTextCallback: ((data: { text: string; timestamp: number }) => void) | null = null;
let narrationToggleResponseCallback: ((data: { status: string; enabled: boolean; message?: string }) => void) | null = null;
let narrationAnalyzeOnceResponseCallback: ((data: NarrationAnalyzeOnceResponse) => void) | null = null;
let narrationSpeakingDoneCallback: (() => void) | null = null;

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
 * Subscribe to one-shot narration analysis responses
 */
export function onNarrationAnalyzeOnceResponse(callback: (data: NarrationAnalyzeOnceResponse) => void): void {
  narrationAnalyzeOnceResponseCallback = callback;
  // Don't re-register the socket listener here — it's registered once at connect time
}

/**
 * Subscribe to narration_speaking_done — emitted by the Pi after Kokoro audio playback finishes
 */
export function onNarrationSpeakingDone(callback: () => void): void {
  narrationSpeakingDoneCallback = callback;
  if (socket) {
    socket.off('narration_speaking_done');
    socket.on('narration_speaking_done', () => {
      console.log('[Socket] 🎙️ Narration speaking done (Pi-side playback finished)');
      if (narrationSpeakingDoneCallback) narrationSpeakingDoneCallback();
    });
  }
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
 * Trigger one immediate AI image analysis + narration pass
 */
export function emitNarrationAnalyzeOnce(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎙️ NARRATION: ANALYZE ONCE`);
    socket.emit('narration_analyze_once', {});
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

// ==========================================
// 🎤 ELEVENLABS TTS SOCKET EVENTS
// ==========================================

export interface ElevenLabsVoice {
  name: string;
  voice_id: string;
}

export interface ElevenLabsValidationResult {
  valid: boolean;
  voices: ElevenLabsVoice[];
  error: string;
}

export interface StartupConfig {
  startup_check_enabled: boolean;
  elevenlabs_api_key_set: boolean;
  elevenlabs_api_key?: string;
  elevenlabs_voice_id: string;
}

export interface StartupCheckResult {
  status: string;
  error?: string;
  critical_ok?: boolean;
  all_ok?: boolean;
  pico_bridge?: boolean;
  front_sonar?: boolean;
  laser?: boolean;
  rear_sonar?: boolean;
  mpu6050?: boolean;
  ir_sensors?: boolean;
  encoder?: boolean;
  battery_voltage?: number;
  motor_current?: number;
}

let elevenLabsValidationResultCallback: ((data: ElevenLabsValidationResult) => void) | null = null;
let startupConfigSyncCallback: ((data: StartupConfig) => void) | null = null;
let startupCheckResultCallback: ((data: StartupCheckResult) => void) | null = null;

/**
 * Subscribe to ElevenLabs API validation results
 */
export function onElevenLabsValidationResult(callback: (data: ElevenLabsValidationResult) => void): void {
  elevenLabsValidationResultCallback = callback;
}

/**
 * Subscribe to startup config sync events (sent on connect)
 */
export function onStartupConfigSync(callback: (data: StartupConfig) => void): void {
  startupConfigSyncCallback = callback;
  if (socket) {
    socket.off('startup_config_sync');
    socket.on('startup_config_sync', (data: StartupConfig) => {
      console.log(`[Socket] ⬛ Startup config sync:`, data);
      if (startupConfigSyncCallback) startupConfigSyncCallback(data);
    });
  }
}

/**
 * Subscribe to startup check results
 */
export function onStartupCheckResult(callback: (data: StartupCheckResult) => void): void {
  startupCheckResultCallback = callback;
  if (socket) {
    socket.off('startup_check_result');
    socket.on('startup_check_result', (data: StartupCheckResult) => {
      console.log(`[Socket] ⬛ Startup check result:`, data);
      if (startupCheckResultCallback) startupCheckResultCallback(data);
    });
  }
}

/**
 * Validate ElevenLabs API key and fetch available voices
 */
export function emitElevenLabsValidateKey(apiKey: string): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎤 ELEVENLABS: Validating API key`);
    socket.emit('elevenlabs_validate_key', { api_key: apiKey });
  }
}

/**
 * Clear the stored ElevenLabs API key on the backend
 */
export function emitElevenLabsKeyClear(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎤 ELEVENLABS: Clear API key`);
    socket.emit('elevenlabs_key_clear', {});
  }
}

/**
 * Update startup check configuration (enabled/disabled, voice ID)
 */
export function emitStartupConfigUpdate(config: { startup_check_enabled?: boolean; elevenlabs_voice_id?: string }): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] ⬛ STARTUP CONFIG UPDATE:`, config);
    socket.emit('startup_config_update', config);
  }
}

/**
 * Trigger manual startup check execution
 */
export function emitStartupCheckRun(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] ⬛ STARTUP: Triggering manual check`);
    socket.emit('startup_check_run', {});
  }
}

/**
 * Request current startup config from backend
 */
export function requestStartupConfig(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] ⬛ STARTUP: Requesting config`);
    socket.emit('startup_config_request', {});
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

// Driver Data callback
export interface DriverData {
  name: string;
  age: number;
}

let driverDataCallback: ((data: DriverData) => void) | null = null;

/**
 * Subscribe to driver data sync events from the backend.
 * Called when driver data is requested or on first connection.
 */
export function onDriverDataSync(callback: (data: DriverData) => void): void {
  driverDataCallback = callback;
  if (socket) {
    socket.off('driver_data_sync');  // avoid duplicate listeners
    socket.on('driver_data_sync', (data: DriverData) => {
      console.log(`[Socket] 👤 Driver data received from backend:`, data);
      if (driverDataCallback) driverDataCallback(data);
    });
  }
}

/**
 * Save driver data to the backend
 * @param name - Driver name
 * @param age - Driver age
 */
export function emitSaveDriverData(name: string, age: number): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 👤 SAVING DRIVER DATA:`, { name, age });
    socket.emit('save_driver_data', { name, age });
  } else {
    console.warn(`[UI Control] ⚠️ Cannot save driver data - socket not connected`);
  }
}

/**
 * Request driver data from the backend
 */
export function requestDriverData(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 👤 Requesting driver data from backend`);
    socket.emit('request_driver_data', {});
  }
}

/**
 * Reset driver data on the backend
 */
export function emitResetDriverData(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 👤 RESETTING DRIVER DATA`);
    socket.emit('reset_driver_data', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot reset driver data - socket not connected`);
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
 * Subscribe to connection state changes (connect/disconnect events)
 * @param callback - Function called with true on connect, false on disconnect
 */
export function onConnectionStateChange(callback: (isConnected: boolean) => void): void {
  connectionStateCallback = callback;
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
 * Emit engine start event
 */
export function emitEngineStart(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 ENGINE: START`);
    socket.emit('engine_start', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit engine start - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit engine stop event
 */
export function emitEngineStop(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 ENGINE: STOP`);
    socket.emit('engine_stop', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit engine stop - socket not connected`, { socket: !!socket, connected: socket?.connected });
  }
}

/**
 * Emit horn control (stateless - just plays horn once)
 */
export function emitHorn(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 HORN: HONK!`);
    socket.emit('horn', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit horn - socket not connected`, { socket: !!socket, connected: socket?.connected });
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
 * Activate Hunter mode (starts follow_target module)
 */
export function emitHunterActivate(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎯 HUNTER: ACTIVATE`);
    socket.emit('hunter_activate', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot activate hunter - socket not connected`);
  }
}

/**
 * Deactivate Hunter mode (stops follow_target module)
 */
export function emitHunterDeactivate(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎯 HUNTER: DEACTIVATE`);
    socket.emit('hunter_deactivate', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot deactivate hunter - socket not connected`);
  }
}

/**
 * Listen for hunter status updates
 */
export function onHunterStatus(callback: (data: { active: boolean }) => void): void {
  if (socket) {
    socket.off('hunter_status');
    socket.on('hunter_status', callback);
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

// ==========================================
// 🎮 GAMEPAD (Console Mode)
// ==========================================

/**
 * Tell the backend to enable gamepad/console mode input processing
 */
export function emitGamepadEnable(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 GAMEPAD: ENABLE`);
    socket.emit('gamepad_enable', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit gamepad enable - socket not connected`);
  }
}

/**
 * Tell the backend to disable gamepad/console mode input processing
 */
export function emitGamepadDisable(): void {
  if (socket && socket.connected) {
    console.log(`[UI Control] 🎮 GAMEPAD: DISABLE`);
    socket.emit('gamepad_disable', {});
  } else {
    console.warn(`[UI Control] ⚠️ Cannot emit gamepad disable - socket not connected`);
  }
}

let gamepadStartPressedCallback: ((data: { engine_running: boolean }) => void) | null = null;

/**
 * Subscribe to gamepad Start button press events
 */
export function onGamepadStartPressed(callback: (data: { engine_running: boolean }) => void): void {
  gamepadStartPressedCallback = callback;
}

let autonomousUpdateCallback: ((data: { autonomous_mode: boolean; source?: string }) => void) | null = null;

/**
 * Subscribe to autonomous_update events (gamepad LB+RB autopilot toggle)
 */
export function onAutonomousUpdate(callback: (data: { autonomous_mode: boolean; source?: string }) => void): void {
  autonomousUpdateCallback = callback;
}

let gamepadHotStartCallback: ((data: { engine_running: boolean; gamepad_enabled: boolean }) => void) | null = null;

/**
 * Subscribe to gamepad Select+Start hot start events
 */
export function onGamepadHotStart(callback: (data: { engine_running: boolean; gamepad_enabled: boolean }) => void): void {
  gamepadHotStartCallback = callback;
}

let gamepadSensorToggleCallback: ((data: { sensor: string; enabled: boolean }) => void) | null = null;

/**
 * Subscribe to gamepad sensor toggle events (Select+A = sonar, Select+X = IR)
 */
export function onGamepadSensorToggle(callback: (data: { sensor: string; enabled: boolean }) => void): void {
  gamepadSensorToggleCallback = callback;
}

let gamepadAutoAccelCallback: ((data: { auto_accel_enabled: boolean; source?: string }) => void) | null = null;

/**
 * Subscribe to gamepad autoaccelerate toggle events (LT+RT)
 */
export function onGamepadAutoAccelUpdate(callback: (data: { auto_accel_enabled: boolean; source?: string }) => void): void {
  gamepadAutoAccelCallback = callback;
}

export default {
  connectToServer,
  disconnectFromServer,
  isConnected,
  onConnectionStateChange,
  onTelemetry,
  emitEngineStart,
  emitEngineStop,
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
  onCameraResponse,
  requestTuning,
  // Narration
  onNarrationConfigSync,
  onNarrationKeyResult,
  onNarrationText,
  onNarrationToggleResponse,
  onNarrationAnalyzeOnceResponse,
  onNarrationSpeakingDone,
  emitNarrationValidateKey,
  emitNarrationConfigUpdate,
  emitNarrationToggle,
  emitNarrationAnalyzeOnce,
  emitNarrationSpeakingDone,
  // Driver Data
  onDriverDataSync,
  emitSaveDriverData,
  requestDriverData,
  emitResetDriverData,
  // ElevenLabs
  onElevenLabsValidationResult,
  emitElevenLabsValidateKey,
  emitElevenLabsKeyClear,
  // Startup Check
  onStartupConfigSync,
  onStartupCheckResult,
  emitStartupConfigUpdate,
  emitStartupCheckRun,
  requestStartupConfig,
  // Gamepad
  emitGamepadEnable,
  emitGamepadDisable,
  onGamepadStartPressed,
  onAutonomousUpdate,
  onGamepadHotStart,
  onGamepadSensorToggle,
  onGamepadAutoAccelUpdate,
  // Hunter
  emitHunterActivate,
  emitHunterDeactivate,
  onHunterStatus,
};
