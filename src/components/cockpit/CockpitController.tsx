import { useState, useCallback, useRef, useEffect } from "react";
import { Header } from "./Header";
import { SteeringWheel } from "./SteeringWheel";
import { CameraFeed } from "./CameraFeed";
import { CarTelemetry } from "./CarTelemetry";
import { GearShifter } from "./GearShifter";
import { HunterPopup } from "./HunterPopup";
import { AutopilotTelemetry, type AutopilotStatus } from "./AutopilotTelemetry";
import { Pedals } from "./Pedals";
import { ImmersiveHUD } from "../ImmersiveHUD";
import { OnboardingScreen } from "../OnboardingScreen";
import { DeviceSelectionScreen, type InputMode } from "../DeviceSelectionScreen";
import * as socketClient from "../../lib/socketClient";
import { ttsService } from "../../lib/ttsService";
import type { AudioOutputDevice } from "../../lib/ttsService";
import { useAutoAcceleration } from "../../hooks/useAutoAcceleration";
import type { SensorStatus } from "./ServiceLight";
import { DEFAULT_TUNING, type TuningConstants } from "./SettingsDialog";
import type { CameraSpecs, NarrationConfig, DriverData } from "../../lib/socketClient";
import { toast } from "@/components/ui/sonner";

interface ControlState {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
  speedMpm: number;     // Real speed in meters per minute (from encoder)
  temperature: number; // CPU temperature in Celsius
  cpuClock: number; // CPU clock speed in MHz
  gpuClock: number; // GPU clock speed in MHz
  rpm: number; // RPM
  encoderAvailable: boolean; // Whether real encoder is available
  batteryVoltage: number; // Battery voltage in V
}

type NetworkMode = "wifi" | "hotspot";

interface NetworkModeSwitchResponse {
  status: string;
  mode: NetworkMode;
  message?: string;
  redirect_url?: string;
}

// Helper function to convert old sensor format to new format
const convertSensorStatus = (
  oldStatus: Record<string, string>
): SensorStatus[] => {
  const sensorNameMap: Record<string, string> = {
    front_sonar: 'Front Sonar (HC-SR04)',
    laser: 'Laser (VL53L0X)',
    left_ir: 'Left IR',
    right_ir: 'Right IR',
    mpu6050: 'MPU6050',
    pico_bridge: 'PICO Bridge',
    camera: 'Camera',
    voltage_sensor: 'Voltage (ADS1115)',
    speaker_amp: 'Speaker + Amp',
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
    speedMpm: 0,
    temperature: 0,
    cpuClock: 0,
    gpuClock: 0,
    rpm: 0,
    encoderAvailable: false,
    batteryVoltage: 0,
  });
  const [isConnected, setIsConnected] = useState(false);
  const defaultWifiLink = "http://raspberrypi.local:5000";
  const [networkMode, setNetworkMode] = useState<NetworkMode>("wifi");
  const [networkSwitching, setNetworkSwitching] = useState(false);
  const [networkLinks, setNetworkLinks] = useState<{ wifi: string; hotspot: string }>({
    wifi: defaultWifiLink,
    hotspot: "http://192.168.4.1:5000",
  });
  const [serverIp, setServerIp] = useState("");
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [isAutoMode, setIsAutoMode] = useState(false);
  const [isIREnabled, setIsIREnabled] = useState(() => {
    const saved = localStorage.getItem('irEnabled');
    return saved !== null ? saved === 'true' : true;
  });
  const [isSonarEnabled, setIsSonarEnabled] = useState(() => {
    const saved = localStorage.getItem('sonarEnabled');
    return saved !== null ? saved === 'true' : true;
  });
  const [isMPU6050Enabled, setIsMPU6050Enabled] = useState(() => {
    const saved = localStorage.getItem('mpu6050Enabled');
    return saved !== null ? saved === 'true' : true;
  });
  const [isRearSonarEnabled, setIsRearSonarEnabled] = useState(() => {
    const saved = localStorage.getItem('rearSonarEnabled');
    return saved !== null ? saved === 'true' : true;
  });
  const [isCameraEnabled, setIsCameraEnabled] = useState(() => {
    // Load camera state from localStorage, default to false (disabled)
    const saved = localStorage.getItem('cameraEnabled');
    return saved === 'true';
  });
  const [isAutopilotEnabled, setIsAutopilotEnabled] = useState(false);
  const [isAutopilotRunning, setIsAutopilotRunning] = useState(false);
  const [isImmersiveView, setIsImmersiveView] = useState(false);
  const [isHunterOpen, setIsHunterOpen] = useState(false);
  const [isHunterActive, setIsHunterActive] = useState(false);
  const [eBrakeActive, setEBrakeActive] = useState(false);
  const [streamUrl, setStreamUrl] = useState<string>("");
  const [isEngineRunning, setIsEngineRunning] = useState(false);
  // Autonomous telemetry state
  const [autonomousState, setAutonomousState] = useState<string>("CRUISING");
  const [sonarDistance, setSonarDistance] = useState<number>(100);
  const [autonomousTargetSpeed, setAutonomousTargetSpeed] = useState<number>(0);
  // Sensor health state
  const [sensors, setSensors] = useState<SensorStatus[]>([
    { name: 'Front Sonar (HC-SR04)', status: 'ok' },
    { name: 'Laser (VL53L0X)', status: 'ok' },
    { name: 'Left IR', status: 'ok' },
    { name: 'Right IR', status: 'ok' },
    { name: 'MPU6050', status: 'ok' },
    { name: 'PICO Bridge', status: 'ok' },
    { name: 'Camera', status: 'ok' },
    { name: 'Voltage (ADS1115)', status: 'ok' },
    { name: 'Speaker + Amp', status: 'ok' },
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
        // MediaMTX-only camera mode does not support integrated CV pipeline.
        base.VISION_ENABLED = false;
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
  // MPU6050 Gyro/Accel telemetry state
  const [gyroZ, setGyroZ] = useState(0);
  const [pidCorrection, setPidCorrection] = useState(0);
  const [gyroAvailable, setGyroAvailable] = useState(false);
  const [gyroCalibrated, setGyroCalibrated] = useState(false);
  // MPU6050 Accelerometer
  const [accelX, setAccelX] = useState(0);
  const [accelY, setAccelY] = useState(0);
  const [accelZ, setAccelZ] = useState(0);
  // Slalom autopilot telemetry
  const [targetYaw, setTargetYaw] = useState(0);
  const [currentHeading, setCurrentHeading] = useState(0);
  const [slalomSign, setSlalomSign] = useState(0);
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
  const [analyzeNowPending, setAnalyzeNowPending] = useState(false);
  const [showAudioUnlockPrompt, setShowAudioUnlockPrompt] = useState(false);
  const [ttsUnlocked, setTtsUnlocked] = useState(false);
  // Driver Data State
  const [driverName, setDriverName] = useState<string | null>(() => {
    try {
      const saved = localStorage.getItem('driverData');
      if (saved) {
        const data = JSON.parse(saved);
        return data.name || null;
      }
    } catch { /* ignore corrupt localStorage */ }
    return null;
  });
  const [driverAge, setDriverAge] = useState<number | null>(() => {
    try {
      const saved = localStorage.getItem('driverData');
      if (saved) {
        const data = JSON.parse(saved);
        return data.age || null;
      }
    } catch { /* ignore corrupt localStorage */ }
    return null;
  });
  const [showOnboarding, setShowOnboarding] = useState<boolean>(() => {
    try {
      const saved = localStorage.getItem('driverData');
      return !saved;
    } catch {
      return true;
    }
  });
  // Input mode: null = show device selection, "console" = view-only (gamepad), "device" = full control
  const [inputMode, setInputMode] = useState<InputMode | null>(() => {
    const saved = localStorage.getItem('inputMode');
    return (saved === 'console' || saved === 'device') ? saved : null;
  });
  const isConsoleMode = inputMode === 'console';
  // Gamepad (console mode) state — driven by telemetry from backend
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const [gamepadGear, setGamepadGear] = useState("1");
  const autoAccelIntervalRef = useRef<number | null>(null);
  const connectionTimeoutRef = useRef<number | null>(null);
  const autoConnectAttemptedRef = useRef(false);
  const pendingGearRef = useRef<{ gear: string; ts: number } | null>(null);
  const analyzeNowTimeoutRef = useRef<number | null>(null);
  const lastMediaMtxUrlRef = useRef<string>("");

  const buildMediaMtxWebRtcUrl = useCallback((ip: string) => {
    return `http://${ip}:8889/rpi_car/`;
  }, []);

  // TTS Browser Unlock: Safari and Chrome require a user gesture before speechSynthesis.speak() works.
  // We use the ttsService singleton which handles voice preloading, unlock, and the
  // Chromium cancel-then-speak race condition (60 ms delay between cancel and speak).
  const unlockTTS = useCallback(() => {
    if (ttsService.isUnlocked) return;
    ttsService.unlock();
    setTtsUnlocked(true);
    setShowAudioUnlockPrompt(false);
  }, []);

  // Wire narration_speaking_done (emitted by Pi after audio playback finishes)
  useEffect(() => {
    socketClient.onNarrationSpeakingDone(() => setNarrationSpeaking(false));
  }, []);

  // Subscribe to socket connection state changes (handles hotspot/no-internet scenarios)
  useEffect(() => {
    socketClient.onConnectionStateChange((connected) => {
      console.log(`🔌 [CockpitController] Connection state changed: ${connected ? '✅ CONNECTED' : '❌ DISCONNECTED'}`);
      setIsConnected(connected);
    });
  }, []);

  // Auto-enable camera when in console (gamepad) mode and connected
  useEffect(() => {
    if (isConsoleMode && isConnected && !isCameraEnabled) {
      console.log('🎮 Console mode: auto-enabling camera');
      socketClient.emitCameraToggle();
    }
  }, [isConsoleMode, isConnected]); // eslint-disable-line react-hooks/exhaustive-deps

  const refreshNetworkMode = useCallback(async () => {
    try {
      const response = await fetch('/api/server-ip', {
        signal: AbortSignal.timeout(5000),
        cache: 'no-store',
      });
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      const data = await response.json() as { ip?: string; transport?: string; hotspot_active?: boolean };
      const resolvedMode: NetworkMode = (data.transport === 'hotspot' || data.hotspot_active) ? 'hotspot' : 'wifi';
      const resolvedIp = typeof data.ip === 'string' ? data.ip.trim() : '';

      setNetworkMode(resolvedMode);
      setNetworkLinks(prev => {
        const next = { ...prev };
        if (resolvedMode === 'hotspot' && resolvedIp) {
          next.hotspot = `http://${resolvedIp}:5000`;
        }
        if (resolvedMode === 'wifi' && resolvedIp && !resolvedIp.startsWith('127.')) {
          next.wifi = `http://${resolvedIp}:5000`;
        }
        return next;
      });
    } catch (error) {
      console.warn('⚠️ [Network] Could not refresh mode from /api/server-ip:', error);
    }
  }, []);

  useEffect(() => {
    refreshNetworkMode();
    const intervalId = window.setInterval(refreshNetworkMode, 15000);
    return () => {
      window.clearInterval(intervalId);
    };
  }, [refreshNetworkMode]);

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

  // Fast auto-connect: discover the right server via lightweight HTTP probes,
  // then open the socket connection only once to the confirmed endpoint.
  useEffect(() => {
    if (!autoConnectAttemptedRef.current) {
      autoConnectAttemptedRef.current = true;
      const smartAutoConnect = async () => {
        // --- Phase 1: discover reachable server via fast HTTP probes ---
        const pageHost = window.location.hostname;
        const isLocalDev = !pageHost || pageHost === 'localhost' || pageHost === '127.0.0.1' || pageHost === '::1';

        interface ProbeCandidate { baseUrl: string; label: string }
        const candidates: ProbeCandidate[] = [];

        if (!isLocalDev) {
          // Production: the page was served by Flask on the Pi — probe the same host
          candidates.push({ baseUrl: `http://${pageHost}:5000`, label: 'page-host' });
        } else {
          // Dev mode: Vite on :8080, backend expected on localhost:5000
          candidates.push({ baseUrl: 'http://localhost:5000', label: 'localhost' });
        }
        // Standard hotspot IP as fallback
        candidates.push({ baseUrl: 'http://192.168.4.1:5000', label: 'hotspot' });

        const probeEndpoint = async (c: ProbeCandidate): Promise<{ ip: string; source: string }> => {
          const resp = await fetch(`${c.baseUrl}/api/server-ip`, { signal: AbortSignal.timeout(3000) });
          if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
          const data = await resp.json();
          return {
            ip: (data.ip as string) ?? new URL(c.baseUrl).hostname,
            source: `${c.label} (${data.transport ?? 'unknown'})`,
          };
        };

        console.log(`🔍 [Discovery] Probing ${candidates.length} endpoint(s) in parallel...`);
        const probes = candidates.map(c => probeEndpoint(c));

        let serverEndpoint: { ip: string; source: string } | null = null;
        try {
          serverEndpoint = await Promise.any(probes);
          console.log(`✅ [Discovery] Server found at ${serverEndpoint.ip} (via ${serverEndpoint.source})`);
        } catch {
          console.error('❌ [Discovery] All endpoint probes failed — no server reachable');
          return;
        }

        // --- Phase 2: connect socket once to the discovered endpoint ---
        const ip = serverEndpoint.ip;
        console.log(`🔌 [Startup] Connecting socket to ${ip}:5000...`);
        try {
          await socketClient.connectToServer(ip, 5000);
          setServerIp(ip);
          setIsConnected(true);
          const url = buildMediaMtxWebRtcUrl(ip);
          lastMediaMtxUrlRef.current = url;
          setStreamUrl(`${url}?t=${Date.now()}`);
          console.log(`✅ [Startup] Connected to ${ip} (discovered via ${serverEndpoint.source})`);
        } catch (error) {
          console.error(`❌ [Startup] Socket connection to ${ip} failed:`, error);
        }
      };
      smartAutoConnect();
    }
  }, [buildMediaMtxWebRtcUrl]);

  // When camera is enabled, force a fresh stream URL so the browser
  // immediately reloads the MediaMTX WebRTC viewer.
  useEffect(() => {
    if (!isCameraEnabled) return;
    setStreamUrl((prev) => {
      const base = prev
        ? prev.split('?')[0]
        : (serverIp ? buildMediaMtxWebRtcUrl(serverIp) : '');
      if (!base) return prev;
      return `${base}?t=${Date.now()}`;
    });
  }, [isCameraEnabled, serverIp, buildMediaMtxWebRtcUrl]);

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

    // Subscribe to sensor config sync from backend (fires on connect)
    // Server is authoritative — update both React state and localStorage
    socketClient.onSensorConfigSync((data) => {
      console.log('📡 Sensor config sync from backend:', data);
      if (data.ir_enabled !== undefined) {
        setIsIREnabled(data.ir_enabled);
        localStorage.setItem('irEnabled', String(data.ir_enabled));
      }
      if (data.engine_running !== undefined) {
        setIsEngineRunning(data.engine_running);
      }
      if (data.sonar_enabled !== undefined) {
        setIsSonarEnabled(data.sonar_enabled);
        localStorage.setItem('sonarEnabled', String(data.sonar_enabled));
      }
      if (data.mpu6050_enabled !== undefined) {
        setIsMPU6050Enabled(data.mpu6050_enabled);
        localStorage.setItem('mpu6050Enabled', String(data.mpu6050_enabled));
      }
      if (data.rear_sonar_enabled !== undefined) {
        setIsRearSonarEnabled(data.rear_sonar_enabled);
        localStorage.setItem('rearSonarEnabled', String(data.rear_sonar_enabled));
      }
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
          VISION_ENABLED: false,
        }));
        // Also update live state so HUD badge reflects confirmed values
        setLiveCameraResolution(data.current_config.resolution);
        setLiveCameraJpegQuality(data.current_config.jpeg_quality);
        setLiveCameraFramerate(data.current_config.framerate);
      }
      if (data.mediamtx?.webrtc_url) {
        const nextBase = data.mediamtx.webrtc_url.trim();
        if (nextBase && nextBase !== lastMediaMtxUrlRef.current) {
          lastMediaMtxUrlRef.current = nextBase;
          setStreamUrl(`${nextBase}${nextBase.includes('?') ? '&' : '?'}t=${Date.now()}`);
        }
      }
    });

    socketClient.onCameraResponse((data) => {
      if (data.camera_enabled !== undefined) {
        setIsCameraEnabled(data.camera_enabled);
        localStorage.setItem('cameraEnabled', String(data.camera_enabled));
      }
      if (data.mediamtx_webrtc_url) {
        const nextBase = data.mediamtx_webrtc_url.trim();
        if (nextBase && nextBase !== lastMediaMtxUrlRef.current) {
          lastMediaMtxUrlRef.current = nextBase;
          setStreamUrl(`${nextBase}${nextBase.includes('?') ? '&' : '?'}t=${Date.now()}`);
        }
      }
      if (data.status === 'error') {
        toast.error('Camera failed to start', {
          description: data.message || data.mediamtx_error || 'MediaMTX pipeline failed',
          duration: 4500,
        });
      }
    });

    // Subscribe to driver data sync from backend
    socketClient.onDriverDataSync((data: DriverData) => {
      console.log('👤 Driver data received from backend:', data);
      setDriverName(data.name);
      setDriverAge(data.age);
      localStorage.setItem('driverData', JSON.stringify(data));
      setShowOnboarding(false);
    });

    // Request driver data from backend when connected
    socketClient.requestDriverData();

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

    // Subscribe to narration text events (AI descriptions)
    // Audio is played on the car (Pi) only — browser shows the text overlay
    socketClient.onNarrationText((data) => {
      console.log(`🎙️ [CockpitController] Narration text received (${data.text?.length} chars): "${data.text?.slice(0, 80)}"`);
      setNarrationLastText(data.text);
      setNarrationSpeaking(true);
    });

    // Subscribe to narration toggle response (image analysis toggle)
    socketClient.onNarrationToggleResponse((data) => {
      if (data.status === 'ok') {
        setImageAnalysisEnabled(data.enabled);
      }
    });

    socketClient.onNarrationAnalyzeOnceResponse((data) => {
      if (analyzeNowTimeoutRef.current) {
        window.clearTimeout(analyzeNowTimeoutRef.current);
        analyzeNowTimeoutRef.current = null;
      }
      setAnalyzeNowPending(false);
      if (data.status === 'error') {
        toast.error('AI analysis failed', {
          description: data.message || 'Could not analyze camera frame.',
          duration: 3500,
        });
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
          speed: data.current_pwm ?? prev.speed,
          speedMpm: data.speed_mpm !== undefined ? data.speed_mpm : prev.speedMpm,
          // Only show system metrics when engine is running
          temperature: isEngineRunning ? (data.temperature ?? prev.temperature) : 0,
          cpuClock: isEngineRunning ? (data.cpu_clock ?? prev.cpuClock) : 0,
          gpuClock: isEngineRunning ? (data.gpu_clock ?? prev.gpuClock) : 0,
          rpm: isEngineRunning ? (data.rpm ?? prev.rpm) : 0,
          batteryVoltage: data.battery_voltage ?? prev.batteryVoltage,
          encoderAvailable: data.encoder_available ?? prev.encoderAvailable,
          // In console (gamepad) mode, sync steering angle from telemetry so the
          // UI steering wheel moves with the physical controller input.
          // In device mode, steeringAngle is controlled by touch — never overwrite.
          ...(isConsoleMode && data.steer_angle !== undefined
            ? { steeringAngle: data.steer_angle }
            : {}),
          // DO NOT update throttle and brake from telemetry - these are user-controlled inputs
          // that must maintain their state while held
        };
      });
      // Update IR enabled state from telemetry (server is authoritative)
      if (data.ir_enabled !== undefined) {
        setIsIREnabled(data.ir_enabled);
        localStorage.setItem('irEnabled', String(data.ir_enabled));
      }
      
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
      if (data.sonar_enabled !== undefined) {
        setIsSonarEnabled(data.sonar_enabled);
        localStorage.setItem('sonarEnabled', String(data.sonar_enabled));
      }
      if (data.mpu6050_enabled !== undefined) {
        setIsMPU6050Enabled(data.mpu6050_enabled);
        localStorage.setItem('mpu6050Enabled', String(data.mpu6050_enabled));
      }
      if (data.rear_sonar_enabled !== undefined) {
        setIsRearSonarEnabled(data.rear_sonar_enabled);
        localStorage.setItem('rearSonarEnabled', String(data.rear_sonar_enabled));
      }
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
      // Update MPU6050 gyro/accel telemetry
      if (data.gyro_z !== undefined) setGyroZ(data.gyro_z);
      if (data.pid_correction !== undefined) setPidCorrection(data.pid_correction);
      if (data.gyro_available !== undefined) setGyroAvailable(data.gyro_available);
      if (data.gyro_calibrated !== undefined) setGyroCalibrated(data.gyro_calibrated);
      if (data.accel_x !== undefined) setAccelX(data.accel_x);
      if (data.accel_y !== undefined) setAccelY(data.accel_y);
      if (data.accel_z !== undefined) setAccelZ(data.accel_z);
      // Update slalom autopilot telemetry
      if (data.target_yaw !== undefined) setTargetYaw(data.target_yaw);
      if (data.current_heading !== undefined) setCurrentHeading(data.current_heading);
      if (data.slalom_sign !== undefined) setSlalomSign(data.slalom_sign);
      // Update narration telemetry
      if (data.narration_enabled !== undefined) setImageAnalysisEnabled(data.narration_enabled);
      if (data.narration_speaking !== undefined) setNarrationSpeaking(data.narration_speaking!);
      // Update gamepad telemetry
      if (data.gamepad_connected !== undefined) setGamepadConnected(data.gamepad_connected);
      if (data.gamepad_gear !== undefined) setGamepadGear(data.gamepad_gear);
      // In console (gamepad) mode, sync emergency brake state from server (gamepad X button)
      if (isConsoleMode && data.emergency_brake_active !== undefined) {
        setEBrakeActive(data.emergency_brake_active);
        setIsEmergencyStop(data.emergency_brake_active);
      }
      // Update live camera config from telemetry (for HUD badge only — does NOT touch tuning state)
      if (data.camera_resolution) setLiveCameraResolution(data.camera_resolution);
      if (data.camera_jpeg_quality !== undefined) setLiveCameraJpegQuality(data.camera_jpeg_quality);
      if (data.camera_framerate !== undefined) setLiveCameraFramerate(data.camera_framerate);
      if (data.camera_mediamtx_webrtc_url) {
        const nextBase = data.camera_mediamtx_webrtc_url.trim();
        if (nextBase && nextBase !== lastMediaMtxUrlRef.current) {
          lastMediaMtxUrlRef.current = nextBase;
          setStreamUrl(`${nextBase}${nextBase.includes('?') ? '&' : '?'}t=${Date.now()}`);
        }
      }
    });

    // Subscribe to gamepad Start button presses (engine toggle from physical controller)
    socketClient.onGamepadStartPressed((data) => {
      console.log('🎮 [Gamepad] Start pressed — engine_running:', data.engine_running);
      setIsEngineRunning(data.engine_running);
      if (!data.engine_running) {
        setIsAutoMode(false);
        setControlState(prev => ({ ...prev, throttle: false, brake: false, speed: 0, speedMpm: 0, temperature: 0, cpuClock: 0, gpuClock: 0, rpm: 0, batteryVoltage: 0 }));
      }
    });

    // Subscribe to gamepad LB+RB autopilot toggle events (instant feedback)
    socketClient.onAutonomousUpdate((data) => {
      console.log('🎮 [Gamepad] Autopilot update — autonomous_mode:', data.autonomous_mode, 'source:', data.source);
      setIsAutopilotRunning(data.autonomous_mode);
      setIsAutoMode(data.autonomous_mode);
      setIsAutopilotEnabled(data.autonomous_mode); // Show/hide AutopilotTelemetry panel
      if (!data.autonomous_mode) {
        // Autopilot disabled — reset related UI state
        setAutonomousState('');
        setAutonomousTargetSpeed(0);
      }
    });

    // Subscribe to gamepad Select+Start hot start events
    socketClient.onGamepadHotStart((data) => {
      console.log('🎮 [Gamepad] Hot Start — engine_running:', data.engine_running, 'gamepad_enabled:', data.gamepad_enabled);
      setIsEngineRunning(data.engine_running);
    });

    // Subscribe to gamepad Select+A / Select+X sensor toggle events
    socketClient.onGamepadSensorToggle((data) => {
      console.log('🎮 [Gamepad] Sensor toggle —', data.sensor, ':', data.enabled);
      if (data.sensor === 'sonar') {
        setIsSonarEnabled(data.enabled);
        localStorage.setItem('sonarEnabled', String(data.enabled));
      } else if (data.sensor === 'ir') {
        setIsIREnabled(data.enabled);
        localStorage.setItem('irEnabled', String(data.enabled));
      }
    });

    // Subscribe to gamepad LT+RT autoaccelerate toggle events
    socketClient.onGamepadAutoAccelUpdate((data) => {
      console.log('🎮 [Gamepad] Auto-accel update — enabled:', data.auto_accel_enabled);
      setIsAutoMode(data.auto_accel_enabled);
    });

    // If console mode is active and connected, tell backend to enable gamepad input
    if (isConsoleMode) {
      socketClient.emitGamepadEnable();
    }

    return () => {
      socketClient.onTelemetry(() => {}); // Unsubscribe
      if (analyzeNowTimeoutRef.current) {
        window.clearTimeout(analyzeNowTimeoutRef.current);
        analyzeNowTimeoutRef.current = null;
      }
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

  const handleOnboardingComplete = useCallback((name: string, age: number) => {
    console.log('✅ Onboarding complete:', { name, age });
    
    // Save to localStorage
    const driverData = { name, age };
    localStorage.setItem('driverData', JSON.stringify(driverData));
    
    // Update state
    setDriverName(name);
    setDriverAge(age);
    setShowOnboarding(false);
    
    // Send to backend if connected
    if (socketClient.isConnected()) {
      socketClient.emitSaveDriverData(name, age);
    }
    
    toast.success(`Welcome, ${name}! 🏁`);
  }, []);

  const handleInputModeSelect = useCallback((mode: InputMode) => {
    setInputMode(mode);
    localStorage.setItem('inputMode', mode);
    if (mode === 'console') {
      // Auto-enable camera in console (gamepad) mode
      if (isConnected && !isCameraEnabled) {
        socketClient.emitCameraToggle();
      }
      // Tell backend to start gamepad bridge
      if (isConnected) {
        socketClient.emitGamepadEnable();
      }
    } else {
      // Switching away from console → disable gamepad bridge
      if (isConnected) {
        socketClient.emitGamepadDisable();
      }
    }
  }, [isConnected, isCameraEnabled]);

  const handleResetDriver = useCallback(() => {
    setShowOnboarding(true);
    setDriverName(null);
    setDriverAge(null);
    // Also reset input mode so device selection appears again after re-onboarding
    setInputMode(null);
    localStorage.removeItem('inputMode');
  }, []);

  const handleThrottleChange = useCallback((active: boolean) => {
    console.log('🎮 Throttle changed:', active, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot throttle');
      return;
    }
    if (!isConnected) {
      console.warn('⚠️ Throttle failed - socket not connected');
      return;
    }
    setControlState(prev => ({ ...prev, throttle: active }));
    socketClient.emitThrottle(active);
  }, [isEngineRunning, isConnected]);

  const handleBrakeChange = useCallback((active: boolean) => {
    console.log('🎮 Brake changed:', active, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot brake');
      return;
    }
    if (!isConnected) {
      console.warn('⚠️ Brake failed - socket not connected');
      return;
    }
    setControlState(prev => ({ ...prev, brake: active }));
    socketClient.emitBrake(active);
  }, [isEngineRunning, isConnected]);

  const handleGearChange = useCallback((gear: string) => {
    console.log('🎮 Gear changed:', gear, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot change gear');
      return;
    }
    if (!isConnected) {
      console.warn('⚠️ Gear change failed - socket not connected');
      return;
    }
    setControlState(prev => ({ ...prev, gear }));
    pendingGearRef.current = { gear, ts: Date.now() };
    socketClient.emitGearChange(gear);
  }, [isEngineRunning, isConnected]);

  const handleLaunch = useCallback(() => {
    if (!isEngineRunning || isAutoMode) return;
    if (!isConnected) {
      console.warn('⚠️ Launch failed - socket not connected');
      return;
    }
    // Launch: auto-accelerate in current gear
    setIsAutoMode(true);
    socketClient.emitAutoAccelEnable();
  }, [isEngineRunning, isAutoMode, isConnected]);

  const handleDonut = useCallback(() => {
    if (!isEngineRunning || isAutoMode) return;
    if (!isConnected) {
      console.warn('⚠️ Donut failed - socket not connected');
      return;
    }
    // Donut: full throttle + steering
    setIsAutoMode(true);
    socketClient.emitThrottle(true);
    socketClient.emitSteering(60);
    socketClient.emitAutoAccelEnable();
  }, [isEngineRunning, isAutoMode, isConnected]);

  const handleEmergencyStop = useCallback(() => {
    const newEmergencyStopState = !isEmergencyStop;
    console.log('🏁 Emergency stop toggled:', newEmergencyStopState);
    
    // Check connection for safety-critical operation
    if (!isConnected) {
      console.warn('⚠️ Emergency stop failed - socket not connected');
      toast.error('Cannot activate emergency stop', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    setIsEmergencyStop(newEmergencyStopState);
    setEBrakeActive(newEmergencyStopState);
    
    if (newEmergencyStopState) {
      // Activating emergency stop
      console.log('🏁 Emergency stop ACTIVE');
      setIsAutoMode(false);
      setIsAutopilotEnabled(false);
      setIsAutopilotRunning(false);
      setControlState(prev => ({ ...prev, speed: 0, speedMpm: 0, throttle: false, brake: false, gear: 'N' }));
      socketClient.emitEmergencyStop();
      socketClient.emitAutoAccelDisable();
    } else {
      // Deactivating emergency stop
      console.log('🏁 Emergency stop RELEASED');
      socketClient.emitEmergencyStopRelease();
    }
  }, [isEmergencyStop, isConnected]);

  // Autopilot-specific e-brake: stops the car but stays in autopilot mode
  const handleAutopilotEBrake = useCallback(() => {
    const newEBrakeState = !eBrakeActive;
    console.log('🛑 Autopilot E-brake toggled:', newEBrakeState);
    
    // Check connection for safety-critical operation
    if (!isConnected) {
      console.warn('⚠️ Autopilot e-brake toggle failed - socket not connected');
      toast.error('Cannot activate e-brake', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    setEBrakeActive(newEBrakeState);
    setIsEmergencyStop(newEBrakeState);
    
    if (newEBrakeState) {
      // Activating e-brake while in autopilot - stop car but remain in autopilot mode
      console.log('🛑 Autopilot E-brake ACTIVE - car stopped, autopilot paused');
      setControlState(prev => ({ ...prev, speed: 0, speedMpm: 0, throttle: false, brake: false }));
      socketClient.emitEmergencyStop();
    } else {
      // Deactivating e-brake while in autopilot - resume autopilot
      console.log('🛑 Autopilot E-brake RELEASED - resuming autopilot');
      socketClient.emitEmergencyStopRelease();
    }
  }, [eBrakeActive, isConnected]);

  const handleEBrakeToggle = useCallback(() => {
    const newEBrakeState = !eBrakeActive;
    console.log('🛑 E-brake toggled:', newEBrakeState);
    
    // Check connection for safety-critical operation
    if (!isConnected) {
      console.warn('⚠️ E-brake toggle failed - socket not connected');
      toast.error('Cannot activate e-brake', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    setEBrakeActive(newEBrakeState);
    
    if (newEBrakeState) {
      // Activating e-brake
      console.log('🛑 E-brake ACTIVE');
      setIsEmergencyStop(true);
      setIsAutopilotEnabled(false);
      setIsAutopilotRunning(false);
      setControlState(prev => ({ ...prev, speed: 0, speedMpm: 0, throttle: false, brake: false, gear: 'N' }));
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
  }, [eBrakeActive, isAutopilotRunning, isConnected]);

  const handleAutoMode = useCallback(() => {
    if (isEmergencyStop) return; // Cannot enable auto mode during emergency stop
    
    // Check connection before toggling auto mode
    if (!isConnected) {
      console.warn('⚠️ Auto mode toggle failed - socket not connected');
      toast.error('Cannot toggle auto mode', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    setIsAutoMode(prev => !prev);
    if (!isAutoMode) {
      socketClient.emitAutoAccelEnable();
    } else {
      socketClient.emitAutoAccelDisable();
    }
  }, [isEmergencyStop, isAutoMode, isConnected]);

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
    setIsSonarEnabled(prev => {
      const newState = !prev;
      localStorage.setItem('sonarEnabled', String(newState));
      return newState;
    });
    socketClient.emitSonarToggle();
  }, [isAutopilotRunning]);

  const handleRearSonarToggle = useCallback(() => {
    console.log('🎮 REAR SONAR sensor toggle');
    if (isAutopilotRunning) {
      console.log('🚫 Rear Sonar cannot be toggled in autonomous mode');
      return;
    }
    setIsRearSonarEnabled(prev => {
      const newState = !prev;
      localStorage.setItem('rearSonarEnabled', String(newState));
      return newState;
    });
    socketClient.emitRearSonarToggle();
  }, [isAutopilotRunning]);

  const handleMPU6050Toggle = useCallback(() => {
    console.log('🎮 MPU6050 sensor toggle');
    if (isAutopilotRunning) {
      console.log('🚫 MPU6050 cannot be toggled in autonomous mode');
      return;
    }
    setIsMPU6050Enabled(prev => {
      const newState = !prev;
      localStorage.setItem('mpu6050Enabled', String(newState));
      return newState;
    });
    socketClient.emitMPU6050Toggle();
  }, [isAutopilotRunning]);

  const handleTargetOpen = useCallback(() => {
    setIsHunterOpen(true);
    setIsHunterActive(false);
    socketClient.emitHunterActivate();
    // Listen for hunter_status to know when subprocess is ready
    socketClient.onHunterStatus((data: { active: boolean }) => {
      setIsHunterActive(data.active);
    });
  }, []);

  const handleHunterClose = useCallback(() => {
    setIsHunterOpen(false);
    setIsHunterActive(false);
    socketClient.emitHunterDeactivate();
  }, []);

  const handleCameraToggle = useCallback(() => {
    console.log('🎮 CAMERA toggle');
    if (!isConnected) {
      toast.error('Cannot toggle camera', {
        description: 'Backend connection lost. Reconnect and try again.',
        duration: 3500,
      });
      return;
    }
    socketClient.emitCameraToggle();
  }, [isConnected]);

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
      if (!isConnected) {
        console.warn('⚠️ Autopilot disable failed - socket not connected');
        return;
      }
      socketClient.emitAutopilotDisable();
    }
    setIsAutopilotEnabled(entering);
  }, [eBrakeActive, isEmergencyStop, isAutopilotEnabled, isAutopilotRunning, isConnected]);

  // Actually start / stop the autopilot FSM on the server
  const handleAutopilotStartStop = useCallback(() => {
    console.log('🎮 Autopilot start/stop, currently running:', isAutopilotRunning);
    if (eBrakeActive || isEmergencyStop) {
      console.log('🚫 Cannot start autopilot while e-brake/emergency stop is active');
      return;
    }
    
    if (!isConnected) {
      console.warn('⚠️ Autopilot control failed - socket not connected');
      toast.error('Cannot control autopilot', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    if (isAutopilotRunning) {
      socketClient.emitAutopilotDisable();
    } else {
      // Check required sensors before enabling autopilot
      const disabledSensors: string[] = [];
      if (!isSonarEnabled) disabledSensors.push('Laser (VL53L0X)');
      if (!isIREnabled) {
        disabledSensors.push('Left IR');
        disabledSensors.push('Right IR');
      }
      if (!isMPU6050Enabled) disabledSensors.push('MPU6050');

      if (disabledSensors.length > 0) {
        toast.error(`Cannot activate Autopilot`, {
          description: `${disabledSensors.join(', ')} ${disabledSensors.length === 1 ? 'is' : 'are'} turned off. Turn ${disabledSensors.length === 1 ? 'it' : 'them'} on first to activate Autopilot.`,
          duration: 5000,
        });
        return;
      }
      socketClient.emitAutopilotEnable();
    }
  }, [eBrakeActive, isEmergencyStop, isAutopilotRunning, isSonarEnabled, isIREnabled, isMPU6050Enabled, isConnected]);

  const handleEngineStart = useCallback(() => {
    console.log('🔧 Engine START button clicked');
    
    // Check connection status before starting engine
    if (!isConnected) {
      console.warn('⚠️ Engine start failed - socket not connected');
      toast.error('Cannot start engine', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    setIsEngineRunning(true);
    socketClient.emitEngineStart();
    console.log('✅ Engine started');
  }, [isConnected]);

  const handleEngineStop = useCallback(() => {
    console.log('🔧 Engine STOP button clicked');
    
    // Check connection status before stopping engine
    if (!isConnected) {
      console.warn('⚠️ Engine stop failed - socket not connected');
      toast.error('Cannot stop engine', {
        description: 'Backend connection lost. Please check your WiFi connection and ensure the RPi is reachable.',
        duration: 4000,
      });
      return;
    }
    
    setIsEngineRunning(false);
    socketClient.emitEngineStop();
    setIsAutoMode(false);
    socketClient.emitAutoAccelDisable();
    socketClient.emitThrottle(false);
    socketClient.emitBrake(true);
    setControlState(prev => ({ ...prev, throttle: false, brake: false, speed: 0, speedMpm: 0, temperature: 0, cpuClock: 0, gpuClock: 0, rpm: 0, batteryVoltage: 0 }));
    console.log('✅ Engine stopped');
  }, [isConnected]);

  const handleHorn = useCallback(() => {
    console.log('📯 Horn button pressed');
    socketClient.emitHorn();
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

  const handleAnalyzeNow = useCallback(() => {
    if (analyzeNowPending) return;
    if (!isConnected) {
      toast.error('Cannot analyze image', {
        description: 'Backend connection lost. Reconnect to trigger AI analysis.',
        duration: 3000,
      });
      return;
    }
    if (!isCameraEnabled) {
      toast.error('Camera is disabled', {
        description: 'Turn on the camera before running AI analysis.',
        duration: 3000,
      });
      return;
    }

    setAnalyzeNowPending(true);
    if (analyzeNowTimeoutRef.current) {
      window.clearTimeout(analyzeNowTimeoutRef.current);
    }
    analyzeNowTimeoutRef.current = window.setTimeout(() => {
      setAnalyzeNowPending(false);
      analyzeNowTimeoutRef.current = null;
      toast.error('AI analysis timed out', {
        description: 'No response received from the server.',
        duration: 3500,
      });
    }, 45000);

    socketClient.emitNarrationAnalyzeOnce();
  }, [analyzeNowPending, isCameraEnabled, isConnected]);

  const handleNetworkModeSwitch = useCallback(async (targetMode: NetworkMode) => {
    if (networkSwitching || targetMode === networkMode) {
      return;
    }

    const fallbackRedirectUrl = targetMode === 'hotspot' ? networkLinks.hotspot : networkLinks.wifi;
    setNetworkSwitching(true);

    try {
      const response = await fetch('/system/switch_network_mode', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ mode: targetMode }),
      });

      let data: Partial<NetworkModeSwitchResponse> = {};
      try {
        data = await response.json() as NetworkModeSwitchResponse;
      } catch {
        data = {};
      }

      if (!response.ok || data.status !== 'ok') {
        throw new Error(data.message || `Request failed with status ${response.status}`);
      }

      const redirectUrl =
        typeof data.redirect_url === 'string' && data.redirect_url.length > 0
          ? data.redirect_url
          : fallbackRedirectUrl;

      toast.success(`Switching to ${targetMode === 'hotspot' ? 'Hotspot' : 'WiFi'}`, {
        description: `Connection will restart. Redirecting to ${redirectUrl}`,
        duration: 6000,
      });

      window.setTimeout(() => {
        window.location.href = redirectUrl;
      }, 1800);
    } catch (error) {
      console.error('❌ [Network] Switch mode failed:', error);
      const message = error instanceof Error ? error.message : 'Unknown error';
      toast.error(`Could not switch to ${targetMode === 'hotspot' ? 'Hotspot' : 'WiFi'}`, {
        description: message,
        duration: 5000,
      });
      setNetworkSwitching(false);
    }
  }, [networkLinks.hotspot, networkLinks.wifi, networkMode, networkSwitching]);

  // Use auto-acceleration hook for client-side auto-throttle
  useAutoAcceleration({
    enabled: isAutoMode,
    gear: controlState.gear,
    isEngineRunning,
    currentSpeed: controlState.speed,
    isBrakePressed: controlState.brake,
  });

  // No-op handlers for console (view-only) mode
  const noopBool = useCallback((_: boolean) => {}, []);
  const noopVoid = useCallback(() => {}, []);
  const noopAngle = useCallback((_: number) => {}, []);
  const noopGear = useCallback((_: string) => {}, []);

  return (
    <>
      {/* Onboarding Screen */}
      {showOnboarding && (
        <OnboardingScreen onComplete={handleOnboardingComplete} />
      )}

      {/* Device Selection Screen (shown after onboarding, before main UI) */}
      {!showOnboarding && inputMode === null && (
        <DeviceSelectionScreen onSelect={handleInputModeSelect} />
      )}

      {/* Main App UI (hidden during onboarding and device selection) */}
      {!showOnboarding && inputMode !== null && (
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
        speedMpm={controlState.speedMpm}
        speedUnit={(tuning.SPEED_UNIT || "m/min") as import("./Speedometer").SpeedUnit}
        gear={controlState.gear}
        throttle={controlState.throttle}
        brake={controlState.brake}
        isConnected={isConnected}
        isAutoMode={isAutoMode}
        isEmergencyStop={isEmergencyStop}
        eBrakeActive={eBrakeActive}
        onThrottleChange={isConsoleMode ? noopBool : handleThrottleChange}
        onBrakeChange={isConsoleMode ? noopBool : handleBrakeChange}
        onEmergencyStop={isConsoleMode ? noopVoid : handleEmergencyStop}
        onEBrakeToggle={isConsoleMode ? noopVoid : handleEBrakeToggle}
        onAutoModeToggle={isConsoleMode ? noopVoid : handleAutoMode}
        onSteeringChange={isConsoleMode ? noopAngle : handleAngleChange}
        onGearChange={isConsoleMode ? noopGear : handleGearChange}
        cameraResolution={liveCameraResolution}
        cameraJpegQuality={liveCameraJpegQuality}
        cameraFramerate={liveCameraFramerate}
        cameraActualFps={cameraActualFps}
        visionActive={visionActive}
        visionFps={visionFps}
        isCameraEnabled={isCameraEnabled}
        userWantsVision={userWantsVision}
        onToggleCamera={isConsoleMode ? noopVoid : handleCameraToggle}
        narrationEnabled={imageAnalysisEnabled}
        narrationSpeaking={narrationSpeaking}
        narrationLastText={narrationLastText}
        steeringAngle={controlState.steeringAngle}
        viewOnly={isConsoleMode}
        gamepadConnected={gamepadConnected}
        isEngineRunning={isEngineRunning}
        inputMode={inputMode}
      />

      {/* Hunter Target Pursuit Popup */}
      <HunterPopup
        isOpen={isHunterOpen}
        onClose={handleHunterClose}
        serverBaseUrl={serverIp ? `http://${serverIp}:5000` : `http://${window.location.hostname}:5000`}
        whepUrl={serverIp ? `http://${serverIp}:8889/rover/whep` : `http://${window.location.hostname}:8889/rover/whep`}
        hunterActive={isHunterActive}
      />
      
      <div className="h-[100dvh] w-full flex flex-col overflow-hidden">
        {/* Header */}
        <Header 
          driverName={driverName || "MACHINE"}
          isConnected={isConnected}
          networkMode={networkMode}
          networkSwitching={networkSwitching}
          networkLinks={networkLinks}
          onNetworkModeSwitch={handleNetworkModeSwitch}
          tuning={tuning}
          onTuningChange={setTuning}
          backendDefaults={backendDefaults}
          cameraSpecs={cameraSpecs}
          narrationConfig={narrationConfig}
          imageAnalysisEnabled={imageAnalysisEnabled}
          onImageAnalysisToggle={handleImageAnalysisToggle}
          ttsUnlocked={ttsUnlocked}
          onUnlockAudio={unlockTTS}
          isIREnabled={isIREnabled}
          isSonarEnabled={isSonarEnabled}
          isMPU6050Enabled={isMPU6050Enabled}
          isRearSonarEnabled={isRearSonarEnabled}
          isCameraEnabled={isCameraEnabled}
          onIRToggle={isConsoleMode ? noopVoid : handleIRToggle}
          onSonarToggle={isConsoleMode ? noopVoid : handleSonarToggle}
          onRearSonarToggle={isConsoleMode ? noopVoid : handleRearSonarToggle}
          onMPU6050Toggle={isConsoleMode ? noopVoid : handleMPU6050Toggle}
          onCameraToggle={isConsoleMode ? noopVoid : handleCameraToggle}
          isAutopilotRunning={isAutopilotRunning}
          onResetDriver={handleResetDriver}
          gamepadConnected={gamepadConnected}
          inputMode={inputMode}
        />
        
        {/* Main Content - Fixed Layout (No Responsive Changes) */}
        <div className="flex-1 flex min-h-0 overflow-hidden flex-row px-4">
          {/* Left Zone: Camera Feed + Steering Wheel */}
          <div className="flex-[0.35] border-r border-border/30 racing-panel m-0.5 flex flex-col overflow-hidden gap-0.5">
            {/* Camera Feed */}
            <div className="h-[30%] min-h-0 overflow-hidden border-none border-b border-border/30">
              <div onClick={handleImmersiveViewToggle} className="cursor-pointer h-full w-full">
                <CameraFeed
                  isConnected={isConnected}
                  streamUrl={streamUrl}
                  isCameraEnabled={isCameraEnabled}
                  cameraResolution={liveCameraResolution}
                  onToggleCamera={isConsoleMode ? noopVoid : handleCameraToggle}
                  narrationEnabled={imageAnalysisEnabled}
                  narrationSpeaking={narrationSpeaking}
                  narrationLastText={narrationLastText}
                  onAnalyzeNow={isConsoleMode ? noopVoid : handleAnalyzeNow}
                  analyzeNowPending={analyzeNowPending}
                />
              </div>
            </div>
            
            {/* Steering Wheel */}
            <div className="flex-1 min-h-0 overflow-hidden flex items-center justify-center">
              <SteeringWheel 
                angle={controlState.steeringAngle} 
                onAngleChange={isConsoleMode ? noopAngle : handleAngleChange}
                isEnabled={isConsoleMode ? false : isEngineRunning}
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
              speedMpm={controlState.speedMpm}
              speedUnit={(tuning.SPEED_UNIT || "m/min") as import("./Speedometer").SpeedUnit}
              temperature={controlState.temperature}
              cpuClock={controlState.cpuClock}
              gpuClock={controlState.gpuClock}
              batteryVoltage={controlState.batteryVoltage}
              rpm={controlState.rpm}
              encoderAvailable={controlState.encoderAvailable}
              onLaunch={isConsoleMode ? noopVoid : handleLaunch}
              onDonut={isConsoleMode ? noopVoid : handleDonut}
              isEngineRunning={isEngineRunning}
              sensors={sensors}
              requiresService={requiresService}
              accelX={accelX}
              accelY={accelY}
              accelZ={accelZ}
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
                targetYaw={targetYaw}
                currentHeading={currentHeading}
                slalomSign={slalomSign}
                gyroZ={gyroZ}
                pidCorrection={pidCorrection}
                gyroAvailable={gyroAvailable}
                gyroCalibrated={gyroCalibrated}
                isMPU6050Enabled={isMPU6050Enabled}
                onEmergencyStop={isConsoleMode ? noopVoid : handleAutopilotEBrake}
                onAutopilotToggle={isConsoleMode ? noopVoid : handleAutopilotToggle}
                onStartStop={isConsoleMode ? noopVoid : handleAutopilotStartStop}
              />
            ) : (
              <GearShifter 
                currentGear={controlState.gear} 
                onGearChange={isConsoleMode ? noopGear : handleGearChange}
                isEmergencyStop={isEmergencyStop}
                isAutoMode={isAutoMode}
                isIREnabled={isIREnabled}
                isSonarEnabled={isSonarEnabled}
                isAutopilotEnabled={isAutopilotEnabled}
                eBrakeActive={eBrakeActive}
                onEmergencyStop={isConsoleMode ? noopVoid : handleEmergencyStop}
                onAutoMode={isConsoleMode ? noopVoid : handleAutoMode}
                onIRToggle={isConsoleMode ? noopVoid : handleIRToggle}
                onSonarToggle={isConsoleMode ? noopVoid : handleSonarToggle}
                onCameraToggle={isConsoleMode ? noopVoid : handleCameraToggle}
                onTargetOpen={isConsoleMode ? noopVoid : handleTargetOpen}
                isCameraEnabled={isCameraEnabled}
                onAutopilotToggle={isConsoleMode ? noopVoid : handleAutopilotToggle}
                isEnabled={isConsoleMode ? false : isEngineRunning}
                isEngineRunning={isEngineRunning}
                onEngineStart={isConsoleMode ? noopVoid : handleEngineStart}
                onEngineStop={isConsoleMode ? noopVoid : handleEngineStop}
                onHorn={isConsoleMode ? noopVoid : handleHorn}
                speed={controlState.speed}
              />
            )}
          </div>
        </div>
        
        {/* Footer Zone: Pedals */}
        <div className="h-[12dvh] min-h-12 max-h-20 border-t border-primary/30 flex-shrink-0 px-4">
          <Pedals 
            onThrottleChange={isConsoleMode ? noopBool : handleThrottleChange}
            onBrakeChange={isConsoleMode ? noopBool : handleBrakeChange}
            isEnabled={isConsoleMode ? false : isEngineRunning}
          />
        </div>
      </div>
    </>
      )}
    </>
  );
};
