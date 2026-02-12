import { useState, useCallback, useRef, useEffect } from "react";
import { Header } from "./Header";
import { SteeringWheel } from "./SteeringWheel";
import { CameraFeed } from "./CameraFeed";
import { CarTelemetry } from "./CarTelemetry";
import { GearShifter } from "./GearShifter";
import { AutopilotTelemetry, type AutopilotStatus } from "./AutopilotTelemetry";
import { Pedals } from "./Pedals";
import { ImmersiveHUD } from "../ImmersiveHUD";
import * as socketClient from "../../lib/socketClient";
import { ttsService } from "../../lib/ttsService";
import type { AudioOutputDevice } from "../../lib/ttsService";
import { useAutoAcceleration } from "../../hooks/useAutoAcceleration";
import type { SensorStatus } from "./ServiceLight";
import { DEFAULT_TUNING, type TuningConstants } from "./SettingsDialog";
import type { CameraSpecs, NarrationConfig } from "../../lib/socketClient";

interface ControlState {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
  temperature: number; // CPU temperature in Celsius
  cpuClock: number; // CPU clock speed in MHz
  gpuClock: number; // GPU clock speed in MHz
  rpm: number; // RPM
}

// Helper function to convert old sensor format to new format
const convertSensorStatus = (
  oldStatus: Record<string, string>
): SensorStatus[] => {
  const sensorNameMap: Record<string, string> = {
    front_sonar: 'Front Sonar',
    rear_sonar: 'Rear Sonar',
    left_ir: 'Left IR',
    right_ir: 'Right IR',
    camera: 'Camera',
  };

  const statusMap: Record<string, 'ok' | 'warning' | 'error'> = {
    'OK': 'ok',
    'WARNING': 'warning',
    'FAILED': 'error',
  };

  return Object.entries(oldStatus).map(([key, value]) => ({
    name: sensorNameMap[key] || key,
    status: statusMap[value.toUpperCase()] || 'ok',
  }));
};

export const CockpitController = () => {
  const [controlState, setControlState] = useState<ControlState>({
    steeringAngle: 0,
    throttle: false,
    brake: false,
    gear: "N",
    speed: 0,
    temperature: 0,
    cpuClock: 0,
    gpuClock: 0,
    rpm: 0,
  });
  const [isConnected, setIsConnected] = useState(false);
  const [serverIp, setServerIp] = useState("");
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [isAutoMode, setIsAutoMode] = useState(false);
  const [isIREnabled, setIsIREnabled] = useState(true);
  const [isSonarEnabled, setIsSonarEnabled] = useState(true);
  const [isCameraEnabled, setIsCameraEnabled] = useState(() => {
    // Load camera state from localStorage, default to false (disabled)
    const saved = localStorage.getItem('cameraEnabled');
    return saved === 'true';
  });
  const [isAutopilotEnabled, setIsAutopilotEnabled] = useState(false);
  const [isAutopilotRunning, setIsAutopilotRunning] = useState(false);
  const [isImmersiveView, setIsImmersiveView] = useState(false);
  const [eBrakeActive, setEBrakeActive] = useState(false);
  const [streamUrl, setStreamUrl] = useState<string>("");
  const [isEngineRunning, setIsEngineRunning] = useState(false);
  // Autonomous telemetry state
  const [autonomousState, setAutonomousState] = useState<string>("CRUISING");
  const [sonarDistance, setSonarDistance] = useState<number>(100);
  const [autonomousTargetSpeed, setAutonomousTargetSpeed] = useState<number>(0);
  // Sensor health state
  const [sensors, setSensors] = useState<SensorStatus[]>([
    { name: 'Front Sonar', status: 'ok' },
    { name: 'Rear Sonar', status: 'ok' },
    { name: 'Left IR', status: 'ok' },
    { name: 'Right IR', status: 'ok' },
    { name: 'Camera', status: 'ok' },
  ]);
  const [requiresService, setRequiresService] = useState(false);
  const [tuning, setTuning] = useState<TuningConstants>(() => {
    // Load persisted camera config from localStorage and merge into defaults
    const base = { ...DEFAULT_TUNING };
    try {
      const saved = localStorage.getItem('cameraConfig');
      if (saved) {
        const config = JSON.parse(saved);
        if (config.resolution) base.CAMERA_RESOLUTION = config.resolution;
        if (config.jpeg_quality) base.CAMERA_JPEG_QUALITY = config.jpeg_quality;
        if (config.framerate) base.CAMERA_FRAMERATE = config.framerate;
        if (config.vision_enabled !== undefined) base.VISION_ENABLED = config.vision_enabled;
      }
    } catch { /* ignore corrupt localStorage */ }
    return base;
  });
  const [backendDefaults, setBackendDefaults] = useState<TuningConstants>(DEFAULT_TUNING);
  const [cameraSpecs, setCameraSpecs] = useState<CameraSpecs>({
    model: "ov5647",
    max_resolution: [2592, 1944],
    supported_modes: ["640x480", "1296x972", "1920x1080", "2592x1944"],
  });
  // Vision / Object Detection state
  const [visionActive, setVisionActive] = useState(false);
  // Live camera config from backend (for HUD badge — separate from tuning to avoid overwriting user edits)
  const [liveCameraResolution, setLiveCameraResolution] = useState<string>(() => {
    try { const s = localStorage.getItem('cameraConfig'); if (s) return JSON.parse(s).resolution || 'low'; } catch {} return 'low';
  });
  const [liveCameraJpegQuality, setLiveCameraJpegQuality] = useState<number>(() => {
    try { const s = localStorage.getItem('cameraConfig'); if (s) return JSON.parse(s).jpeg_quality || 70; } catch {} return 70;
  });
  const [liveCameraFramerate, setLiveCameraFramerate] = useState<number>(() => {
    try { const s = localStorage.getItem('cameraConfig'); if (s) return JSON.parse(s).framerate || 30; } catch {} return 30;
  });
  const [cameraObstacleDistance, setCameraObstacleDistance] = useState(999);
  const [cameraDetectionsCount, setCameraDetectionsCount] = useState(0);
  const [cameraInPathCount, setCameraInPathCount] = useState(0);
  const [cameraClosestObject, setCameraClosestObject] = useState("");
  const [cameraClosestConfidence, setCameraClosestConfidence] = useState(0);
  const [visionFps, setVisionFps] = useState(0);
  const [userWantsVision, setUserWantsVision] = useState(false);
  const [cameraActualFps, setCameraActualFps] = useState(0);
  // AI Image Analysis state (backend-synced: whether AI is analyzing camera frames)
  // Persisted across sessions — will auto-toggle on backend after connection
  const [imageAnalysisEnabled, setImageAnalysisEnabled] = useState(() => {
    const saved = localStorage.getItem('imageAnalysisEnabled');
    return saved === 'true';
  });
  // AI Narration state
  const [narrationConfig, setNarrationConfig] = useState<NarrationConfig>({
    provider: 'gemini',
    api_key_set: false,
    api_key_masked: '',
    model: '',
    interval: 8,
    enabled: false,
  });
  const [narrationSpeaking, setNarrationSpeaking] = useState(false);
  const [narrationLastText, setNarrationLastText] = useState('');
  const [showAudioUnlockPrompt, setShowAudioUnlockPrompt] = useState(false);
  const [ttsUnlocked, setTtsUnlocked] = useState(false);
  const autoAccelIntervalRef = useRef<number | null>(null);
  const connectionTimeoutRef = useRef<number | null>(null);
  const autoConnectAttemptedRef = useRef(false);
  const pendingGearRef = useRef<{ gear: string; ts: number } | null>(null);

  // TTS Browser Unlock: Safari and Chrome require a user gesture before speechSynthesis.speak() works.
  // We use the ttsService singleton which handles voice preloading, unlock, and the
  // Chromium cancel-then-speak race condition (60 ms delay between cancel and speak).
  const unlockTTS = useCallback(() => {
    if (ttsService.isUnlocked) return;
    ttsService.unlock();
    setTtsUnlocked(true);
    setShowAudioUnlockPrompt(false);
  }, []);

  // Wire ttsService speaking-state changes back into React state
  useEffect(() => {
    ttsService.onSpeakingChange((speaking) => setNarrationSpeaking(speaking));
    ttsService.onDone(() => socketClient.emitNarrationSpeakingDone());
  }, []);

  useEffect(() => {
    const handleFirstInteraction = () => {
      unlockTTS();
      document.removeEventListener('touchstart', handleFirstInteraction, true);
      document.removeEventListener('click', handleFirstInteraction, true);
    };
    document.addEventListener('touchstart', handleFirstInteraction, { capture: true, once: true });
    document.addEventListener('click', handleFirstInteraction, { capture: true, once: true });
    return () => {
      document.removeEventListener('touchstart', handleFirstInteraction, true);
      document.removeEventListener('click', handleFirstInteraction, true);
    };
  }, [unlockTTS]);

  // Smart auto-connect: try localhost first, then fallback to server IP
  useEffect(() => {
    if (!autoConnectAttemptedRef.current) {
      autoConnectAttemptedRef.current = true;
      const smartAutoConnect = async () => {
        const tryConnection = async (ip: string) => {
          console.log(`🔌 [Startup] Attempting connection to ${ip}:5000...`);
          try {
            await socketClient.connectToServer(ip, 5000);
            setServerIp(ip);
            setIsConnected(true);
            setStreamUrl(`http://${ip}:5000/video_feed`);
            console.log(`✅ [Startup] Successfully connected to ${ip}`);
            return true;
          } catch (error) {
            console.warn(`⚠️ [Startup] Failed to connect to ${ip}:`, error);
            return false;
          }
        };

        // Try localhost first (for local development)
        const localhostConnected = await tryConnection('localhost');
        
        if (!localhostConnected) {
          // If localhost fails, fetch the server's network IP
          console.log('🔍 [Startup] Localhost failed, fetching server IP...');
          try {
            // Try to get server IP from current origin
            const protocol = window.location.protocol;
            const hostname = window.location.hostname;
            const port = window.location.port;
            const apiUrl = `${protocol}//${hostname}${port ? ':' + port : ''}/api/server-ip`;
            const response = await fetch(apiUrl);
            const data = await response.json();
            console.log(`📡 [Startup] Server IP from API: ${data.ip}`);
            await tryConnection(data.ip);
          } catch (error) {
            console.error('❌ [Startup] Could not fetch server IP:', error);
          }
        }
      };
      smartAutoConnect();
    }
  }, []);

  // Sync userWantsVision with tuning.VISION_ENABLED
  useEffect(() => {
    setUserWantsVision(tuning.VISION_ENABLED);
  }, [tuning.VISION_ENABLED]);

  // Setup telemetry subscription when connected
  useEffect(() => {
    if (!isConnected) {
      return;
    }

    // Subscribe to tuning sync from backend (fires on connect + after tuning_request)
    socketClient.onTuningSync((data) => {
      console.log('⚙️ Tuning sync from backend:', data);
      if (data.tuning) {
        // Merge backend autopilot tuning into state, but PRESERVE camera settings
        // because _active_tuning from the backend only contains autopilot constants
        setTuning(prev => ({
          ...prev,
          ...(data.tuning as unknown as Partial<TuningConstants>),
          // Keep camera settings from current state (they come from localStorage / camera_config_response)
          CAMERA_RESOLUTION: prev.CAMERA_RESOLUTION,
          CAMERA_JPEG_QUALITY: prev.CAMERA_JPEG_QUALITY,
          CAMERA_FRAMERATE: prev.CAMERA_FRAMERATE,
          VISION_ENABLED: prev.VISION_ENABLED,
        }));
      }
      if (data.defaults) {
        setBackendDefaults(data.defaults as unknown as TuningConstants);
      }
    });

    // Subscribe to camera specs sync from backend (fires on connect)
    socketClient.onCameraSpecsSync((data: CameraSpecs) => {
      console.log('📷 Camera specs sync from backend:', data);
      setCameraSpecs(data);
    });

    // Subscribe to camera config response (confirms applied settings)
    socketClient.onCameraConfigResponse((data) => {
      console.log('📷 Camera config response:', data);
      if (data.current_config) {
        setTuning(prev => ({
          ...prev,
          CAMERA_RESOLUTION: data.current_config.resolution,
          CAMERA_JPEG_QUALITY: data.current_config.jpeg_quality,
          CAMERA_FRAMERATE: data.current_config.framerate,
        }));
        // Also update live state so HUD badge reflects confirmed values
        setLiveCameraResolution(data.current_config.resolution);
        setLiveCameraJpegQuality(data.current_config.jpeg_quality);
        setLiveCameraFramerate(data.current_config.framerate);
      }
    });

    // Re-apply persisted camera config to backend on connect
    try {
      const saved = localStorage.getItem('cameraConfig');
      if (saved) {
        const config = JSON.parse(saved);
        console.log('📷 Re-applying persisted camera config:', config);
        socketClient.emitCameraConfigUpdate({
          resolution: config.resolution,
          jpeg_quality: config.jpeg_quality,
          framerate: config.framerate,
        });
        // Also toggle vision if it was enabled
        if (config.vision_enabled) {
          // Vision toggle is a toggle, so we only send if we want it enabled
          // The backend will report the actual state via telemetry
        }
      }
    } catch { /* ignore */ }

    // Also explicitly request in case the connect event was missed
    socketClient.requestTuning();

    // Subscribe to narration config sync from backend (fires on connect)
    socketClient.onNarrationConfigSync((data: NarrationConfig) => {
      console.log('🎙️ Narration config sync from backend:', data);
      setNarrationConfig(data);

      // Restore persisted image analysis state: if user had it on but backend is off, re-enable
      const savedAnalysis = localStorage.getItem('imageAnalysisEnabled') === 'true';
      if (savedAnalysis && data.api_key_set && data.model && !data.enabled) {
        console.log('🎙️ Auto-restoring image analysis from saved preference');
        socketClient.emitNarrationToggle(true);
        setImageAnalysisEnabled(true);
      } else {
        setImageAnalysisEnabled(data.enabled);
        // Keep localStorage in sync with backend truth on first connect
        localStorage.setItem('imageAnalysisEnabled', String(data.enabled));
      }
    });

    // Subscribe to narration text events (AI descriptions) and play via browser TTS
    socketClient.onNarrationText((data) => {
      console.log(`🎙️ [CockpitController] Narration text received (${data.text?.length} chars): "${data.text?.slice(0, 80)}"`);
      setNarrationLastText(data.text);

      // Always speak via TTS when image analysis is enabled
      console.log(`🎙️ [CockpitController] ✅ TTS playback, ttsService.isUnlocked=${ttsService.isUnlocked}, ttsService.isSupported=${ttsService.isSupported}`);

      // Show unlock prompt if TTS hasn't been unlocked yet via user gesture
      if (!ttsService.isUnlocked) {
        console.log('🎙️ [CockpitController] ⚠️ TTS NOT unlocked yet — showing unlock prompt');
        setShowAudioUnlockPrompt(true);
        setNarrationSpeaking(true);
        return;
      }

      // Speak via the TTS service (handles cancel → delay → speak,
      // voice selection, Chrome keep-alive, and GC protection)
      console.log('🎙️ [CockpitController] Calling ttsService.speak()...');
      ttsService.speak(data.text);
    });

    // Subscribe to narration toggle response (image analysis toggle)
    socketClient.onNarrationToggleResponse((data) => {
      if (data.status === 'ok') {
        setImageAnalysisEnabled(data.enabled);
      }
    });

    // Subscribe to telemetry updates
    // Note: throttle, brake, and steeringAngle are NOT updated from telemetry - they're controlled only by user input
    // This ensures continuous hold behavior works correctly (the server won't overwrite the UI state)
    socketClient.onTelemetry((data) => {
      // Gear bounce-back guard: compute outside the state updater to avoid
      // side-effects (ref mutation) inside React's updater function, which
      // breaks under Strict Mode double-invocation and concurrent rendering.
      const pending = pendingGearRef.current;
      let gearOverride: string | null = null;
      if (pending) {
        if (data.gear === pending.gear) {
          // Backend confirmed the pending gear change — clear guard
          pendingGearRef.current = null;
        } else {
          // Stale telemetry arrived before backend echoed our change — keep optimistic value
          gearOverride = pending.gear;
        }
      }

      setControlState(prev => {
        return {
          ...prev,
          gear: gearOverride ?? data.gear ?? prev.gear,
          speed: data.current_pwm || prev.speed,
          // Only show system metrics when engine is running
          temperature: isEngineRunning ? (data.temperature || prev.temperature) : 0,
          cpuClock: isEngineRunning ? (data.cpu_clock || prev.cpuClock) : 0,
          gpuClock: isEngineRunning ? (data.gpu_clock || prev.gpuClock) : 0,
          rpm: isEngineRunning ? (data.rpm || prev.rpm) : 0,
          // DO NOT update throttle and brake from telemetry - these are user-controlled inputs
          // that must maintain their state while held
        };
      });
      // Update IR enabled state from telemetry
      setIsIREnabled(data.ir_enabled ?? true);
      
      // Update camera enabled state from telemetry
      if (data.camera_enabled !== undefined) {
        setIsCameraEnabled(data.camera_enabled);
        localStorage.setItem('cameraEnabled', String(data.camera_enabled));
      }
      
      // Update autonomous telemetry
      if (data.autonomous_mode !== undefined) setIsAutopilotRunning(data.autonomous_mode);
      if (data.autonomous_state) setAutonomousState(data.autonomous_state);
      if (data.sonar_distance !== undefined) setSonarDistance(data.sonar_distance);
      if (data.autonomous_target_speed !== undefined) setAutonomousTargetSpeed(data.autonomous_target_speed);
      if (data.sonar_enabled !== undefined) setIsSonarEnabled(data.sonar_enabled);
      // Update sensor health status
      if (data.sensor_status) {
        const convertedSensors = convertSensorStatus(data.sensor_status);
        setSensors(convertedSensors);
        const hasError = convertedSensors.some(s => s.status !== 'ok');
        setRequiresService(hasError);
      }
      // Update vision / object detection telemetry
      if (data.vision_active !== undefined) setVisionActive(data.vision_active);
      if (data.camera_obstacle_distance !== undefined) setCameraObstacleDistance(data.camera_obstacle_distance);
      if (data.camera_detections_count !== undefined) setCameraDetectionsCount(data.camera_detections_count);
      if (data.camera_in_path_count !== undefined) setCameraInPathCount(data.camera_in_path_count);
      if (data.camera_closest_object !== undefined) setCameraClosestObject(data.camera_closest_object);
      if (data.camera_closest_confidence !== undefined) setCameraClosestConfidence(data.camera_closest_confidence);
      if (data.vision_fps !== undefined) setVisionFps(data.vision_fps);
      if (data.camera_actual_fps !== undefined) setCameraActualFps(data.camera_actual_fps);
      // Update narration telemetry
      if (data.narration_enabled !== undefined) setImageAnalysisEnabled(data.narration_enabled);
      if (data.narration_speaking !== undefined) setNarrationSpeaking(prev => prev || data.narration_speaking!);
      // Update live camera config from telemetry (for HUD badge only — does NOT touch tuning state)
      if (data.camera_resolution) setLiveCameraResolution(data.camera_resolution);
      if (data.camera_jpeg_quality !== undefined) setLiveCameraJpegQuality(data.camera_jpeg_quality);
      if (data.camera_framerate !== undefined) setLiveCameraFramerate(data.camera_framerate);
    });

    return () => {
      socketClient.onTelemetry(() => {}); // Unsubscribe
      // Clean up TTS on unmount
      if (window.speechSynthesis.speaking) {
        window.speechSynthesis.cancel();
      }
    };
  }, [isConnected, isEngineRunning]);



  const handleAngleChange = useCallback((angle: number) => {
    console.log('🎮 Steering angle changed:', angle, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot steer');
      return;
    }
    setControlState(prev => ({ ...prev, steeringAngle: angle }));
    socketClient.emitSteering(Math.round(angle));
  }, [isEngineRunning]);

  const handleThrottleChange = useCallback((active: boolean) => {
    console.log('🎮 Throttle changed:', active, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot throttle');
      return;
    }
    setControlState(prev => ({ ...prev, throttle: active }));
    socketClient.emitThrottle(active);
  }, [isEngineRunning]);

  const handleBrakeChange = useCallback((active: boolean) => {
    console.log('🎮 Brake changed:', active, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot brake');
      return;
    }
    setControlState(prev => ({ ...prev, brake: active }));
    socketClient.emitBrake(active);
  }, [isEngineRunning]);

  const handleGearChange = useCallback((gear: string) => {
    console.log('🎮 Gear changed:', gear, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot change gear');
      return;
    }
    setControlState(prev => ({ ...prev, gear }));
    pendingGearRef.current = { gear, ts: Date.now() };
    socketClient.emitGearChange(gear);
  }, [isEngineRunning]);

  const handleLaunch = useCallback(() => {
    if (!isEngineRunning || isAutoMode) return;
    // Launch: auto-accelerate in current gear
    setIsAutoMode(true);
    socketClient.emitAutoAccelEnable();
  }, [isEngineRunning, isAutoMode]);

  const handleDonut = useCallback(() => {
    if (!isEngineRunning || isAutoMode) return;
    // Donut: full throttle + steering
    setIsAutoMode(true);
    socketClient.emitThrottle(true);
    socketClient.emitSteering(60);
    socketClient.emitAutoAccelEnable();
  }, [isEngineRunning, isAutoMode]);

  const handleEmergencyStop = useCallback(() => {
    const newEmergencyStopState = !isEmergencyStop;
    console.log('🏁 Emergency stop toggled:', newEmergencyStopState);
    setIsEmergencyStop(newEmergencyStopState);
    setEBrakeActive(newEmergencyStopState);
    
    if (newEmergencyStopState) {
      // Activating emergency stop
      console.log('🏁 Emergency stop ACTIVE');
      setIsAutoMode(false);
      setIsAutopilotEnabled(false);
      setIsAutopilotRunning(false);
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
      socketClient.emitEmergencyStop();
      socketClient.emitAutoAccelDisable();
    } else {
      // Deactivating emergency stop
      console.log('🏁 Emergency stop RELEASED');
      socketClient.emitEmergencyStopRelease();
    }
  }, [isEmergencyStop]);

  // Autopilot-specific e-brake: stops the car but stays in autopilot mode
  const handleAutopilotEBrake = useCallback(() => {
    const newEBrakeState = !eBrakeActive;
    console.log('🛑 Autopilot E-brake toggled:', newEBrakeState);
    setEBrakeActive(newEBrakeState);
    setIsEmergencyStop(newEBrakeState);
    
    if (newEBrakeState) {
      // Activating e-brake while in autopilot - stop car but remain in autopilot mode
      console.log('🛑 Autopilot E-brake ACTIVE - car stopped, autopilot paused');
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false }));
      socketClient.emitEmergencyStop();
    } else {
      // Deactivating e-brake while in autopilot - resume autopilot
      console.log('🛑 Autopilot E-brake RELEASED - resuming autopilot');
      socketClient.emitEmergencyStopRelease();
    }
  }, [eBrakeActive]);

  const handleEBrakeToggle = useCallback(() => {
    const newEBrakeState = !eBrakeActive;
    console.log('🛑 E-brake toggled:', newEBrakeState);
    setEBrakeActive(newEBrakeState);
    
    if (newEBrakeState) {
      // Activating e-brake
      console.log('🛑 E-brake ACTIVE');
      setIsEmergencyStop(true);
      setIsAutopilotEnabled(false);
      setIsAutopilotRunning(false);
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
      socketClient.emitEmergencyStop();
      if (isAutopilotRunning) {
        socketClient.emitAutopilotDisable();
      }
    } else {
      // Deactivating e-brake
      console.log('🛑 E-brake RELEASED');
      setIsEmergencyStop(false);
      socketClient.emitEmergencyStopRelease();
    }
  }, [eBrakeActive, isAutopilotRunning]);

  const handleAutoMode = useCallback(() => {
    if (isEmergencyStop) return; // Cannot enable auto mode during emergency stop
    setIsAutoMode(prev => !prev);
    if (!isAutoMode) {
      socketClient.emitAutoAccelEnable();
    } else {
      socketClient.emitAutoAccelDisable();
    }
  }, [isEmergencyStop, isAutoMode]);

  const handleIRToggle = useCallback(() => {
    console.log('🎮 IR sensor toggle');
    if (isAutopilotRunning) {
      console.log('🚫 IR sensors cannot be toggled in autonomous mode');
      return;
    }
    socketClient.emitIRToggle();
  }, [isAutopilotRunning]);

  const handleSonarToggle = useCallback(() => {
    console.log('🎮 SONAR sensor toggle');
    if (isAutopilotRunning) {
      console.log('🚫 Sonar sensor cannot be toggled in autonomous mode');
      return;
    }
    setIsSonarEnabled(prev => !prev);
    socketClient.emitSonarToggle();
  }, [isAutopilotRunning]);

  const handleCameraToggle = useCallback(() => {
    console.log('🎮 CAMERA toggle');
    // Toggle local state immediately for instant feedback
    setIsCameraEnabled(prev => {
      const newState = !prev;
      localStorage.setItem('cameraEnabled', String(newState));
      return newState;
    });
    socketClient.emitCameraToggle();
  }, []);

  // Toggle autopilot VIEW (does not start/stop the autopilot FSM)
  const handleAutopilotToggle = useCallback(() => {
    console.log('🎮 Autopilot view toggle');
    if (eBrakeActive || isEmergencyStop) {
      console.log('🚫 Cannot open autopilot while e-brake/emergency stop is active');
      return;
    }
    const entering = !isAutopilotEnabled;
    if (!entering && isAutopilotRunning) {
      // Exiting autopilot view while running → stop the autopilot first
      socketClient.emitAutopilotDisable();
    }
    setIsAutopilotEnabled(entering);
  }, [eBrakeActive, isEmergencyStop, isAutopilotEnabled, isAutopilotRunning]);

  // Actually start / stop the autopilot FSM on the server
  const handleAutopilotStartStop = useCallback(() => {
    console.log('🎮 Autopilot start/stop, currently running:', isAutopilotRunning);
    if (eBrakeActive || isEmergencyStop) {
      console.log('🚫 Cannot start autopilot while e-brake/emergency stop is active');
      return;
    }
    if (isAutopilotRunning) {
      socketClient.emitAutopilotDisable();
    } else {
      socketClient.emitAutopilotEnable();
    }
  }, [eBrakeActive, isEmergencyStop, isAutopilotRunning]);

  const handleEngineStart = useCallback(() => {
    console.log('🔧 Engine START button clicked');
    setIsEngineRunning(true);
    console.log('✅ Engine started');
  }, []);

  const handleEngineStop = useCallback(() => {
    console.log('🔧 Engine STOP button clicked');
    setIsEngineRunning(false);
    setIsAutoMode(false);
    socketClient.emitAutoAccelDisable();
    socketClient.emitThrottle(false);
    socketClient.emitBrake(true);
    setControlState(prev => ({ ...prev, throttle: false, brake: false, speed: 0, temperature: 0, cpuClock: 0, gpuClock: 0, rpm: 0 }));
    console.log('✅ Engine stopped');
  }, []);

  const handleImmersiveViewToggle = useCallback(() => {
    setIsImmersiveView(prev => !prev);
  }, []);

  const handleImageAnalysisToggle = useCallback((enabled: boolean) => {
    setImageAnalysisEnabled(enabled);
    localStorage.setItem('imageAnalysisEnabled', String(enabled));
    socketClient.emitNarrationToggle(enabled);
    if (!enabled) {
      // Stop TTS and clear text when analysis is disabled
      if (window.speechSynthesis.speaking) {
        window.speechSynthesis.cancel();
      }
      setNarrationSpeaking(false);
      setNarrationLastText('');
    }
  }, []);

  // Use auto-acceleration hook for client-side auto-throttle
  useAutoAcceleration({
    enabled: isAutoMode,
    gear: controlState.gear,
    isEngineRunning,
    currentSpeed: controlState.speed,
    isBrakePressed: controlState.brake,
  });

  return (
    <>
      {/* Audio Unlock Prompt - shown when narration text arrives but TTS hasn't been unlocked */}
      {showAudioUnlockPrompt && (
        <div 
          className="fixed inset-0 z-[9998] flex items-center justify-center bg-background/70 backdrop-blur-sm cursor-pointer"
          onClick={() => {
            unlockTTS();
          }}
          onTouchStart={() => {
            unlockTTS();
          }}
        >
          <div className="flex flex-col items-center gap-3 p-6 rounded-xl bg-card border border-primary/50 shadow-lg max-w-xs text-center">
            <div className="w-16 h-16 rounded-full bg-primary/20 border border-primary/50 flex items-center justify-center animate-pulse">
              <svg className="w-8 h-8 text-primary" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5" />
                <path d="M19.07 4.93a10 10 0 0 1 0 14.14" />
                <path d="M15.54 8.46a5 5 0 0 1 0 7.07" />
              </svg>
            </div>
            <div className="text-sm racing-text text-foreground font-bold">TAP TO ENABLE AUDIO</div>
            <div className="text-xs text-muted-foreground">Browser requires a tap to allow AI voice narration</div>
          </div>
        </div>
      )}

      {/* Immersive HUD Overlay */}
      <ImmersiveHUD
        isOpen={isImmersiveView}
        onClose={handleImmersiveViewToggle}
        streamUrl={streamUrl}
        speed={controlState.speed}
        gear={controlState.gear}
        throttle={controlState.throttle}
        brake={controlState.brake}
        isConnected={isConnected}
        isAutoMode={isAutoMode}
        isEmergencyStop={isEmergencyStop}
        eBrakeActive={eBrakeActive}
        onThrottleChange={handleThrottleChange}
        onBrakeChange={handleBrakeChange}
        onEmergencyStop={handleEmergencyStop}
        onEBrakeToggle={handleEBrakeToggle}
        onAutoModeToggle={handleAutoMode}
        onSteeringChange={handleAngleChange}
        onGearChange={handleGearChange}
        cameraResolution={liveCameraResolution}
        cameraJpegQuality={liveCameraJpegQuality}
        cameraFramerate={liveCameraFramerate}
        cameraActualFps={cameraActualFps}
        visionActive={visionActive}
        visionFps={visionFps}
        isCameraEnabled={isCameraEnabled}
        userWantsVision={userWantsVision}
        onToggleCamera={handleCameraToggle}
        narrationEnabled={imageAnalysisEnabled}
        narrationSpeaking={narrationSpeaking}
        narrationLastText={narrationLastText}
      />
      
      <div className="h-[100dvh] w-full flex flex-col overflow-hidden">
        {/* Header */}
        <Header 
          isConnected={isConnected}
          tuning={tuning}
          onTuningChange={setTuning}
          backendDefaults={backendDefaults}
          cameraSpecs={cameraSpecs}
          narrationConfig={narrationConfig}
          imageAnalysisEnabled={imageAnalysisEnabled}
          onImageAnalysisToggle={handleImageAnalysisToggle}
          ttsUnlocked={ttsUnlocked}
          onUnlockAudio={unlockTTS}
        />
        
        {/* Main Content - Fixed Layout (No Responsive Changes) */}
        <div className="flex-1 flex min-h-0 overflow-hidden flex-row px-4">
          {/* Left Zone: Camera Feed + Steering Wheel */}
          <div className="flex-[0.35] border-r border-border/30 racing-panel m-0.5 flex flex-col overflow-hidden gap-0.5">
            {/* Camera Feed */}
            <div className="h-[30%] min-h-0 p-0.5 border-none border-b border-border/30">
              <div onClick={handleImmersiveViewToggle} className="cursor-pointer h-full w-full">
                <CameraFeed isConnected={isConnected} streamUrl={streamUrl} isCameraEnabled={isCameraEnabled} onToggleCamera={handleCameraToggle} narrationEnabled={imageAnalysisEnabled} narrationSpeaking={narrationSpeaking} narrationLastText={narrationLastText} />
              </div>
            </div>
            
            {/* Steering Wheel */}
            <div className="flex-1 min-h-0 overflow-hidden flex items-center justify-center">
              <SteeringWheel 
                angle={controlState.steeringAngle} 
                onAngleChange={handleAngleChange}
                isEnabled={isEngineRunning}
              />
            </div>
          </div>
          
          {/* Center Zone: Car Telemetry */}
          <div className="flex-[0.4] racing-panel m-0.5 overflow-hidden">
            <CarTelemetry 
              steeringAngle={controlState.steeringAngle}
              throttle={controlState.throttle}
              brake={controlState.brake}
              gear={controlState.gear}
              speed={controlState.speed}
              temperature={controlState.temperature}
              cpuClock={controlState.cpuClock}
              gpuClock={controlState.gpuClock}
              rpm={controlState.rpm}
              onLaunch={handleLaunch}
              onDonut={handleDonut}
              isEngineRunning={isEngineRunning}
              sensors={sensors}
              requiresService={requiresService}
            />
          </div>
          
          {/* Right Zone: Gear Shifter or Autopilot Telemetry */}
          <div className="flex-[0.25] border-l border-border/30 racing-panel m-0.5 overflow-hidden">
            {isAutopilotEnabled ? (
              <AutopilotTelemetry
                status={autonomousState as AutopilotStatus}
                accelerationPercent={autonomousTargetSpeed}
                distanceToObstacle={sonarDistance}
                eBrakeActive={eBrakeActive}
                isRunning={isAutopilotRunning}
                visionActive={visionActive}
                cameraObstacleDistance={cameraObstacleDistance}
                cameraDetectionsCount={cameraDetectionsCount}
                cameraInPathCount={cameraInPathCount}
                cameraClosestObject={cameraClosestObject}
                cameraClosestConfidence={cameraClosestConfidence}
                visionFps={visionFps}
                onEmergencyStop={handleAutopilotEBrake}
                onAutopilotToggle={handleAutopilotToggle}
                onStartStop={handleAutopilotStartStop}
              />
            ) : (
              <GearShifter 
                currentGear={controlState.gear} 
                onGearChange={handleGearChange}
                isEmergencyStop={isEmergencyStop}
                isAutoMode={isAutoMode}
                isIREnabled={isIREnabled}
                isSonarEnabled={isSonarEnabled}
                isAutopilotEnabled={isAutopilotEnabled}
                eBrakeActive={eBrakeActive}
                onEmergencyStop={handleEmergencyStop}
                onAutoMode={handleAutoMode}
                onIRToggle={handleIRToggle}
                onSonarToggle={handleSonarToggle}
                onCameraToggle={handleCameraToggle}
                isCameraEnabled={isCameraEnabled}
                onAutopilotToggle={handleAutopilotToggle}
                isEnabled={isEngineRunning}
                isEngineRunning={isEngineRunning}
                onEngineStart={handleEngineStart}
                onEngineStop={handleEngineStop}
              />
            )}
          </div>
        </div>
        
        {/* Footer Zone: Pedals */}
        <div className="h-[12dvh] min-h-12 max-h-20 border-t border-primary/30 flex-shrink-0 px-4">
          <Pedals 
            onThrottleChange={handleThrottleChange}
            onBrakeChange={handleBrakeChange}
            isEnabled={isEngineRunning}
          />
        </div>
      </div>
    </>
  );
};
