import { useState, useCallback, useRef, useEffect } from "react";
import { createPortal } from "react-dom";
import { Settings, X, Minus, Plus, ChevronDown, ChevronRight, HelpCircle, Loader2, Check, Eye, EyeOff, Mic, MicOff, Trash2 } from "lucide-react";
import * as socketClient from "../../lib/socketClient";
import type { CameraSpecs, NarrationConfig, NarrationModel, NarrationKeyResult, KokoroValidationResult } from "../../lib/socketClient";
import { toast } from "@/components/ui/sonner";

export interface TuningConstants {
  // Navigation
  BASE_SPEED: number;
  ESCAPE_SPEED: number;
  GYRO_KP: number;
  // Distances
  CRITICAL_DIST: number;
  WARN_DIST: number;
  // Slalom dodge
  SLALOM_BASE_DEG: number;
  SLALOM_PROXIMITY: number;
  SLALOM_COOLDOWN: number;
  SLALOM_HEADING_THRESH: number;
  SLALOM_CLEAR_TIME: number;
  YAW_CENTER_RATE: number;
  YAW_CENTER_DEAD: number;
  // Motor trims
  FL_TRIM: number;
  FR_TRIM: number;
  RL_TRIM: number;
  RR_TRIM: number;
  // Escape timing
  ESCAPE_STOP_TIME: number;
  ESCAPE_REVERSE_TIME: number;
  ESCAPE_SPIN_TIME: number;
  ESCAPE_RESUME_TIME: number;
  // Rear sonar
  REAR_CLEAR_CM: number;
  // Gyro
  CALIBRATION_TIME: number;
  STATUS_INTERVAL: number;
  // Camera & Vision Settings (non-autopilot, kept for camera config)
  CAMERA_RESOLUTION: string;
  CAMERA_JPEG_QUALITY: number;
  CAMERA_FRAMERATE: number;
  VISION_ENABLED: boolean;
  // Speedometer display unit
  SPEED_UNIT: string;
}

export const DEFAULT_TUNING: TuningConstants = {
  // Navigation
  BASE_SPEED: 50,
  ESCAPE_SPEED: 65,
  GYRO_KP: 1.5,
  // Distances
  CRITICAL_DIST: 20,
  WARN_DIST: 100,
  // Slalom dodge
  SLALOM_BASE_DEG: 15,
  SLALOM_PROXIMITY: 0.30,
  SLALOM_COOLDOWN: 0.5,
  SLALOM_HEADING_THRESH: 10,
  SLALOM_CLEAR_TIME: 1.0,
  YAW_CENTER_RATE: 0.2,
  YAW_CENTER_DEAD: 2.0,
  // Motor trims
  FL_TRIM: 0.6,
  FR_TRIM: 0.6,
  RL_TRIM: 1.0,
  RR_TRIM: 1.0,
  // Escape timing
  ESCAPE_STOP_TIME: 0.1,
  ESCAPE_REVERSE_TIME: 0.8,
  ESCAPE_SPIN_TIME: 0.5,
  ESCAPE_RESUME_TIME: 0.2,
  // Rear sonar
  REAR_CLEAR_CM: 20,
  // Gyro
  CALIBRATION_TIME: 2.0,
  STATUS_INTERVAL: 0.5,
  // Camera (non-autopilot)
  CAMERA_RESOLUTION: "640x480",
  CAMERA_JPEG_QUALITY: 70,
  CAMERA_FRAMERATE: 30,
  VISION_ENABLED: false,
  // Speedometer
  SPEED_UNIT: "m/min",
};

interface ParamConfig {
  key: keyof TuningConstants;
  label: string;
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  info: string;
  type?: "number" | "boolean" | "select";
  options?: Array<{ value: string | number | boolean; label: string }>;
}

const TUNING_GROUPS: { title: string; params: ParamConfig[] }[] = [
  {
    title: "CAMERA & VISION",
    params: [
      {
        key: "CAMERA_RESOLUTION",
        label: "Resolution",
        unit: "",
        type: "select",
        options: [
          { value: "640x480", label: "640×480 (Low)" },
          { value: "1280x720", label: "1280×720 (Medium)" },
          { value: "1920x1080", label: "1920×1080 (High)" },
        ],
        info: "Camera resolution. Higher resolution = better quality but slower streaming. Changes take effect immediately. ↑ Increase: better detail, slower FPS. ↓ Decrease: faster streaming, lower quality.",
      },
      {
        key: "CAMERA_JPEG_QUALITY",
        label: "JPEG Quality",
        min: 10,
        max: 100,
        step: 5,
        unit: "%",
        info: "JPEG compression quality for streaming. Higher = better quality but more bandwidth. ↑ Increase: sharper image, more data. ↓ Decrease: more compression, faster streaming.",
      },
      {
        key: "CAMERA_FRAMERATE",
        label: "Framerate",
        min: 5,
        max: 60,
        step: 5,
        unit: "FPS",
        info: "Camera capture framerate. Higher FPS = smoother video but more CPU usage. Changes take effect on next camera restart. ↑ Increase: smoother motion. ↓ Decrease: less CPU load.",
      },
      {
        key: "VISION_ENABLED",
        label: "CV Toggle",
        unit: "",
        type: "boolean",
        info: "Enable/disable computer vision object detection. When ON, uses MobileNetSSD for real-time object recognition. Requires camera to be enabled.",
      },
    ],
  },
  {
    title: "NAVIGATION",
    params: [
      { key: "BASE_SPEED", label: "Base Speed", min: 10, max: 100, step: 5, unit: "%", info: "Cruising PWM duty cycle. ↑ Increase: faster driving. ↓ Decrease: slower, more cautious." },
      { key: "ESCAPE_SPEED", label: "Escape Speed", min: 20, max: 100, step: 5, unit: "%", info: "PWM during reverse/spin escape maneuvers. ↑ Increase: faster escapes. ↓ Decrease: gentler escape." },
      { key: "GYRO_KP", label: "Gyro Kp", min: 0.1, max: 5.0, step: 0.1, unit: "", info: "Proportional gain for gyro yaw correction. Higher = snappier turns. ↑ Increase: more aggressive steering. ↓ Decrease: smoother but slower correction." },
    ],
  },
  {
    title: "DISTANCE THRESHOLDS",
    params: [
      { key: "CRITICAL_DIST", label: "Critical Distance", min: 5, max: 50, step: 1, unit: "cm", info: "Sonar distance that triggers P1 escape maneuver. ↑ Increase: reacts earlier. ↓ Decrease: gets closer before escaping." },
      { key: "WARN_DIST", label: "Warning Distance", min: 30, max: 200, step: 5, unit: "cm", info: "Sonar distance that triggers P2 slalom dodge. ↑ Increase: starts dodging earlier. ↓ Decrease: approaches closer before dodging." },
      { key: "REAR_CLEAR_CM", label: "Rear Clearance", min: 5, max: 50, step: 1, unit: "cm", info: "Minimum rear clearance to allow reversing during escape. ↑ Increase: needs more space. ↓ Decrease: reverses in tighter spaces." },
    ],
  },
  {
    title: "SLALOM DODGE",
    params: [
      { key: "SLALOM_BASE_DEG", label: "Base Angle", min: 5, max: 45, step: 1, unit: "°", info: "Base yaw dodge increment per trigger. ↑ Increase: sharper initial dodge. ↓ Decrease: gentler initial dodge." },
      { key: "SLALOM_PROXIMITY", label: "Proximity Factor", min: 0.05, max: 1.0, step: 0.05, unit: "", info: "Multiplier for distance-based dynamic increment. Closer = bigger dodge. ↑ Increase: more aggressive close-range. ↓ Decrease: more uniform dodging." },
      { key: "SLALOM_COOLDOWN", label: "Cooldown", min: 0.1, max: 2.0, step: 0.1, unit: "s", info: "Minimum time between successive dodge increments. ↑ Increase: calmer dodging. ↓ Decrease: more rapid yaw accumulation." },
      { key: "SLALOM_HEADING_THRESH", label: "Heading Threshold", min: 2, max: 30, step: 1, unit: "°", info: "Heading error below which a new dodge increment is added. ↑ Increase: adds dodge sooner. ↓ Decrease: waits for more accuracy." },
      { key: "SLALOM_CLEAR_TIME", label: "Clear Time", min: 0.5, max: 5.0, step: 0.5, unit: "s", info: "Time after last dodge before resetting dodge direction. ↑ Increase: remembers dodge longer. ↓ Decrease: resets sooner." },
      { key: "YAW_CENTER_RATE", label: "Center Rate", min: 0.05, max: 1.0, step: 0.05, unit: "°/tick", info: "Rate at which target yaw returns to center when clear. ↑ Increase: snaps to center faster. ↓ Decrease: gradual centering." },
      { key: "YAW_CENTER_DEAD", label: "Center Deadzone", min: 0.5, max: 10.0, step: 0.5, unit: "°", info: "Yaw deadzone — no centering correction within this range. ↑ Increase: wider tolerance. ↓ Decrease: tighter tracking." },
    ],
  },
  {
    title: "MOTOR TRIMS",
    params: [
      { key: "FL_TRIM", label: "Front-Left", min: 0.1, max: 1.5, step: 0.05, unit: "×", info: "PWM multiplier for front-left motor. Compensates for motor differences. ↑ Increase: more power. ↓ Decrease: less power." },
      { key: "FR_TRIM", label: "Front-Right", min: 0.1, max: 1.5, step: 0.05, unit: "×", info: "PWM multiplier for front-right motor. ↑ Increase: more power. ↓ Decrease: less power." },
      { key: "RL_TRIM", label: "Rear-Left", min: 0.1, max: 1.5, step: 0.05, unit: "×", info: "PWM multiplier for rear-left motor. ↑ Increase: more power. ↓ Decrease: less power." },
      { key: "RR_TRIM", label: "Rear-Right", min: 0.1, max: 1.5, step: 0.05, unit: "×", info: "PWM multiplier for rear-right motor. ↑ Increase: more power. ↓ Decrease: less power." },
    ],
  },
  {
    title: "ESCAPE TIMING",
    params: [
      { key: "ESCAPE_STOP_TIME", label: "Stop Pause", min: 0.0, max: 1.0, step: 0.05, unit: "s", info: "Pause before reversing during escape. ↑ Increase: longer pause. ↓ Decrease: faster reaction." },
      { key: "ESCAPE_REVERSE_TIME", label: "Reverse Duration", min: 0.2, max: 3.0, step: 0.1, unit: "s", info: "How long the car reverses during escape. ↑ Increase: reverses further. ↓ Decrease: shorter reverse." },
      { key: "ESCAPE_SPIN_TIME", label: "Spin Duration", min: 0.2, max: 2.0, step: 0.1, unit: "s", info: "How long the car spins during escape. ↑ Increase: wider turn. ↓ Decrease: smaller rotation." },
      { key: "ESCAPE_RESUME_TIME", label: "Resume Pause", min: 0.0, max: 1.0, step: 0.05, unit: "s", info: "Pause after spin before resuming cruise. ↑ Increase: more settling time. ↓ Decrease: faster resumption." },
    ],
  },
  {
    title: "GYRO",
    params: [
      { key: "CALIBRATION_TIME", label: "Calibration", min: 0.5, max: 5.0, step: 0.5, unit: "s", info: "Gyro calibration duration at autopilot start. More time = better drift offset. ↑ Increase: more accurate. ↓ Decrease: faster startup." },
      { key: "STATUS_INTERVAL", label: "Status Log", min: 0.1, max: 5.0, step: 0.1, unit: "s", info: "Interval between console status prints. ↑ Increase: less log spam. ↓ Decrease: more detailed logging." },
    ],
  },
  {
    title: "SPEEDOMETER",
    params: [
      {
        key: "SPEED_UNIT",
        label: "Speed Unit",
        unit: "",
        type: "select",
        options: [
          { value: "m/min", label: "m/min (meters per minute)" },
          { value: "cm/s", label: "cm/s (centimeters per second)" },
          { value: "km/h", label: "km/h (kilometers per hour)" },
          { value: "mph", label: "mph (miles per hour)" },
        ],
        info: "Display unit for the speedometer gauge. Speed is measured from the rear-right wheel encoder (20-hole disc, 65mm wheel).",
      },
    ],
  },
];

// CV mode high-quality camera settings constants
const CV_HIGH_QUALITY_SETTINGS = {
  resolution: "1920x1080",
  jpeg_quality: 70,
  framerate: 60,
} as const;

interface SettingsDialogProps {
  tuning: TuningConstants;
  onTuningChange: (tuning: TuningConstants) => void;
  backendDefaults?: TuningConstants;
  cameraSpecs?: CameraSpecs;
  // AI Narration props
  narrationConfig?: NarrationConfig;
  imageAnalysisEnabled?: boolean;
  onImageAnalysisToggle?: (enabled: boolean) => void;
  // Sensor toggle props
  isIREnabled?: boolean;
  isSonarEnabled?: boolean;
  isMPU6050Enabled?: boolean;
  isRearSonarEnabled?: boolean;
  isCameraEnabled?: boolean;
  onIRToggle?: () => void;
  onSonarToggle?: () => void;
  onRearSonarToggle?: () => void;
  onMPU6050Toggle?: () => void;
  onCameraToggle?: () => void;
  isAutopilotRunning?: boolean;
  // Driver reset
  onResetDriver?: () => void;
}

/* ------------------------------------------------------------------ */
/*  ParamRow – single tuning parameter with +/- buttons & info toggle */
/* ------------------------------------------------------------------ */
const ParamRow = ({
  config,
  value,
  onChange,
  backendDefault,
}: {
  config: ParamConfig;
  value: number | string | boolean;
  onChange: (val: number | string | boolean) => void;
  backendDefault: number | string | boolean;
}) => {
  const [showInfo, setShowInfo] = useState(false);

  /* ---------- boolean toggle ---------- */
  if (config.type === "boolean") {
    return (
      <div className="py-0.5">
        <div className="flex items-center justify-between gap-1">
          <div className="flex items-center gap-1 flex-1 min-w-0">
            <button
              onClick={() => setShowInfo(!showInfo)}
              className={`flex-shrink-0 p-0 transition-colors ${showInfo ? "text-primary" : "text-muted-foreground/50 hover:text-primary/70"}`}
            >
              <HelpCircle className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            </button>
            <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text min-w-0 truncate">
              {config.label}
            </span>
          </div>
          <button
            onClick={() => onChange(!value)}
            className={`px-3 py-0.5 rounded border text-[10px] sm:text-xs racing-text transition-colors ${
              value
                ? "border-primary bg-primary/20 text-primary hover:bg-primary/30"
                : "border-border bg-muted/30 text-muted-foreground hover:bg-muted/50"
            }`}
          >
            {value ? "ON" : "OFF"}
          </button>
        </div>
        {showInfo && (
          <div className="mt-1 ml-3 p-1.5 rounded bg-primary/5 border border-primary/20 text-[7px] sm:text-[9px] text-muted-foreground leading-relaxed">
            {config.info}
            <div className="mt-0.5 text-primary/70 font-semibold">
              DEFAULT: {backendDefault ? "ON" : "OFF"}
            </div>
          </div>
        )}
      </div>
    );
  }

  /* ---------- select dropdown ---------- */
  if (config.type === "select" && config.options) {
    return (
      <div className="py-0.5">
        <div className="flex items-center justify-between gap-1">
          <div className="flex items-center gap-1 flex-1 min-w-0">
            <button
              onClick={() => setShowInfo(!showInfo)}
              className={`flex-shrink-0 p-0 transition-colors ${showInfo ? "text-primary" : "text-muted-foreground/50 hover:text-primary/70"}`}
            >
              <HelpCircle className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            </button>
            <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text min-w-0 truncate">
              {config.label}
            </span>
          </div>
          <select
            value={String(value)}
            onChange={(e) => onChange(e.target.value)}
            className="px-2 py-0.5 rounded border border-border bg-card text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none max-w-[160px]"
          >
            {config.options.map((opt) => (
              <option key={String(opt.value)} value={String(opt.value)}>
                {opt.label}
              </option>
            ))}
          </select>
        </div>
        {showInfo && (
          <div className="mt-1 ml-3 p-1.5 rounded bg-primary/5 border border-primary/20 text-[7px] sm:text-[9px] text-muted-foreground leading-relaxed">
            {config.info}
            <div className="mt-0.5 text-primary/70 font-semibold">
              DEFAULT: {String(backendDefault)}
            </div>
          </div>
        )}
      </div>
    );
  }

  /* ---------- number (default) ---------- */
  const numValue = typeof value === "number" ? value : typeof backendDefault === "number" ? backendDefault : 0;
  const clamp = (v: number) => Math.min(config.max!, Math.max(config.min!, v));
  const decimals = (config.step ?? 1) < 1 ? 1 : 0;
  const defaultVal = backendDefault as number;

  const handleManualChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const raw = e.target.value;
    if (raw === "" || raw === "-") return;
    const parsed = parseFloat(raw);
    if (!isNaN(parsed)) {
      onChange(clamp(parsed));
    }
  };

  return (
    <div className="py-0.5">
      <div className="flex items-center justify-between gap-1">
        <div className="flex items-center gap-1 flex-1 min-w-0">
          <button
            onClick={() => setShowInfo(!showInfo)}
            className={`flex-shrink-0 p-0 transition-colors ${showInfo ? "text-primary" : "text-muted-foreground/50 hover:text-primary/70"}`}
          >
            <HelpCircle className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
          </button>
          <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text flex-1 min-w-0 truncate">
            {config.label}
          </span>
        </div>
        <div className="flex items-center gap-0.5">
          <button
            onClick={() => onChange(clamp(+(numValue - (config.step ?? 1)).toFixed(2)))}
            disabled={numValue <= (config.min ?? 0)}
            className="w-5 h-5 sm:w-6 sm:h-6 rounded border border-border bg-muted flex items-center justify-center text-foreground hover:border-primary/50 hover:bg-primary/10 transition-colors touch-feedback disabled:opacity-30 disabled:pointer-events-none"
          >
            <Minus className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
          </button>
          <input
            type="number"
            value={Number(numValue.toFixed(decimals))}
            onChange={handleManualChange}
            min={config.min}
            max={config.max}
            step={config.step}
            className="w-12 sm:w-14 h-5 sm:h-6 bg-card border border-border rounded px-1 text-center text-[10px] sm:text-xs text-foreground racing-number focus:border-primary focus:outline-none [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
          />
          <button
            onClick={() => onChange(clamp(+(numValue + (config.step ?? 1)).toFixed(2)))}
            disabled={numValue >= (config.max ?? 100)}
            className="w-5 h-5 sm:w-6 sm:h-6 rounded border border-border bg-muted flex items-center justify-center text-foreground hover:border-primary/50 hover:bg-primary/10 transition-colors touch-feedback disabled:opacity-30 disabled:pointer-events-none"
          >
            <Plus className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
          </button>
          {config.unit && (
            <span className="text-[8px] sm:text-[10px] text-muted-foreground racing-text w-4 text-right">
              {config.unit}
            </span>
          )}
        </div>
      </div>
      {showInfo && (
        <div className="mt-1 ml-3 p-1.5 rounded bg-primary/5 border border-primary/20 text-[7px] sm:text-[9px] text-muted-foreground leading-relaxed">
          {config.info}
          <div className="mt-0.5 text-primary/70 font-semibold">
            DEFAULT: {defaultVal}
            {config.unit ? ` ${config.unit}` : ""} · RANGE: {config.min}–{config.max}
          </div>
        </div>
      )}
    </div>
  );
};

/* ------------------------------------------------------------------ */
/*  SettingsRow – a generic key/value row matching new design language */
/* ------------------------------------------------------------------ */
const SettingsRow = ({
  label,
  info,
  children,
}: {
  label: string;
  info?: string;
  children: React.ReactNode;
}) => {
  const [showInfo, setShowInfo] = useState(false);

  return (
    <div className="py-0.5">
      <div className="flex items-center justify-between gap-1">
        <div className="flex items-center gap-1 flex-1 min-w-0">
          {info && (
            <button
              onClick={() => setShowInfo(!showInfo)}
              className={`flex-shrink-0 p-0 transition-colors ${showInfo ? "text-primary" : "text-muted-foreground/50 hover:text-primary/70"}`}
            >
              <HelpCircle className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            </button>
          )}
          <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text min-w-0 truncate">
            {label}
          </span>
        </div>
        {children}
      </div>
      {showInfo && info && (
        <div className="mt-1 ml-3 p-1.5 rounded bg-primary/5 border border-primary/20 text-[7px] sm:text-[9px] text-muted-foreground leading-relaxed">
          {info}
        </div>
      )}
    </div>
  );
};

/* ------------------------------------------------------------------ */
/*  CollapsibleGroup                                                  */
/* ------------------------------------------------------------------ */
const CollapsibleGroup = ({
  title,
  children,
  defaultOpen = false,
}: {
  title: string;
  children: React.ReactNode;
  defaultOpen?: boolean;
}) => {
  const [isOpen, setIsOpen] = useState(defaultOpen);

  return (
    <div className="border border-border/50 rounded bg-card/50">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="w-full flex items-center justify-between px-2 py-1 hover:bg-muted/30 transition-colors"
      >
        <span className="text-[8px] sm:text-[10px] racing-text text-primary">{title}</span>
        {isOpen ? (
          <ChevronDown className="w-3 h-3 text-muted-foreground" />
        ) : (
          <ChevronRight className="w-3 h-3 text-muted-foreground" />
        )}
      </button>
      {isOpen && <div className="px-2 pb-1.5 space-y-0">{children}</div>}
    </div>
  );
};

/* ================================================================== */
/*  SettingsDialog                                                    */
/* ================================================================== */
export const SettingsDialog = ({
  tuning,
  onTuningChange,
  backendDefaults = DEFAULT_TUNING,
  cameraSpecs,
  narrationConfig,
  imageAnalysisEnabled = false,
  onImageAnalysisToggle,
  isIREnabled = true,
  isSonarEnabled = true,
  isMPU6050Enabled = true,
  isRearSonarEnabled = true,
  isCameraEnabled = false,
  onIRToggle,
  onSonarToggle,
  onRearSonarToggle,
  onMPU6050Toggle,
  onCameraToggle,
  isAutopilotRunning = false,
  onResetDriver,
}: SettingsDialogProps) => {
  const [isOpen, setIsOpen] = useState(false);
  const [synced, setSynced] = useState(false);

  /* ---- AI Narration local state ---- */
  const [narrationProvider, setNarrationProvider] = useState(narrationConfig?.provider || "gemini");
  const [apiKeyInput, setApiKeyInput] = useState("");
  const [showApiKey, setShowApiKey] = useState(false);
  const [isValidatingKey, setIsValidatingKey] = useState(false);
  const [isKeyValid, setIsKeyValid] = useState(narrationConfig?.api_key_set || false);
  const [availableModels, setAvailableModels] = useState<NarrationModel[]>([]);
  const [selectedModel, setSelectedModel] = useState(narrationConfig?.model || "");
  const [narrationInterval, setNarrationInterval] = useState(narrationConfig?.interval || 30);
  const [keyError, setKeyError] = useState("");

  /* ---- Kokoro TTS local state ---- */
  const [kokoroEnabled, setKokoroEnabled] = useState(false);
  const [kokoroIp, setKokoroIp] = useState("");
  const [kokoroVoice, setKokoroVoice] = useState("");
  const [availableKokoroVoices, setAvailableKokoroVoices] = useState<string[]>([]);
  const [isValidatingKokoro, setIsValidatingKokoro] = useState(false);
  const [kokoroError, setKokoroError] = useState("");

  /* ---- Startup Check local state ---- */
  const [startupCheckEnabled, setStartupCheckEnabled] = useState(true);
  const [elevenLabsApiKeyInput, setElevenLabsApiKeyInput] = useState("");
  const [isValidatingElevenLabs, setIsValidatingElevenLabs] = useState(false);
  const [isElevenLabsKeyValid, setIsElevenLabsKeyValid] = useState(false);
  const [availableElevenLabsVoices, setAvailableElevenLabsVoices] = useState<Array<{ name: string; voice_id: string }>>([]);
  const [selectedElevenLabsVoice, setSelectedElevenLabsVoice] = useState("JBFqnCBsd6RMkjVDRZzb");
  const [elevenLabsError, setElevenLabsError] = useState("");
  const [isRunningStartupCheck, setIsRunningStartupCheck] = useState(false);

  // Sync narration config from backend when it changes
  useEffect(() => {
    if (narrationConfig) {
      setNarrationProvider(narrationConfig.provider || "gemini");
      setIsKeyValid(narrationConfig.api_key_set);
      setSelectedModel(narrationConfig.model || "");
      setNarrationInterval(narrationConfig.interval || 30);
      if (narrationConfig.api_key_set && narrationConfig.api_key_masked) {
        setApiKeyInput(narrationConfig.api_key_masked);
      } else if (!narrationConfig.api_key_set) {
        setApiKeyInput("");
      }
      if (narrationConfig.models && narrationConfig.models.length > 0) {
        setAvailableModels(narrationConfig.models);
      }
      if (narrationConfig.kokoro_enabled !== undefined) {
        setKokoroEnabled(narrationConfig.kokoro_enabled || false);
      }
      if (narrationConfig.kokoro_ip) {
        setKokoroIp(narrationConfig.kokoro_ip);
      }
      if (narrationConfig.kokoro_voice) {
        setKokoroVoice(narrationConfig.kokoro_voice);
      }
      if (narrationConfig.kokoro_voices && narrationConfig.kokoro_voices.length > 0) {
        setAvailableKokoroVoices(narrationConfig.kokoro_voices);
      }
      // Sync ElevenLabs settings from narration config
      if (narrationConfig.elevenlabs_api_key_set !== undefined) {
        setIsElevenLabsKeyValid(narrationConfig.elevenlabs_api_key_set);
        if (!narrationConfig.elevenlabs_api_key_set) {
          setElevenLabsApiKeyInput("");
        }
      }
      if (typeof narrationConfig.elevenlabs_api_key === "string") {
        setElevenLabsApiKeyInput(narrationConfig.elevenlabs_api_key);
      }
      if (narrationConfig.elevenlabs_voice_id) {
        setSelectedElevenLabsVoice(narrationConfig.elevenlabs_voice_id);
      }
    }
  }, [narrationConfig]);

  // Subscribe to socket events
  useEffect(() => {
    socketClient.onNarrationKeyResult((data: NarrationKeyResult) => {
      console.log("🎙️ SettingsDialog received key result:", data);
      setIsValidatingKey(false);
      if (data.valid) {
        setIsKeyValid(true);
        setAvailableModels(data.models);
        setKeyError("");
        toast.success("🔑 API Key Validated", {
          description: `${data.models.length} multimodal models available`,
          duration: 3000,
        });
        if (data.models.length > 0) {
          const firstModel = data.models[0].name;
          setSelectedModel(firstModel);
          socketClient.emitNarrationConfigUpdate({ model: firstModel });
        }
      } else {
        setIsKeyValid(false);
        setAvailableModels([]);
        setKeyError(data.error || "Invalid API key");
        toast.error("❌ API Key Invalid", {
          description: data.error || "Could not validate the API key",
          duration: 4000,
        });
      }
    });

    socketClient.onNarrationToggleResponse((data) => {
      if (data.status === "error" && data.message) {
        toast.error("🎙️ Narration Error", {
          description: data.message,
          duration: 4000,
        });
      }
    });

    // Subscribe to narration config sync (sent after validation or on reconnect)
    socketClient.onNarrationConfigSync((data) => {
      console.log("🎙️ SettingsDialog received narration config sync:", data);
      setNarrationProvider(data.provider || "gemini");
      setIsKeyValid(data.api_key_set || false);
      setSelectedModel(data.model || "");
      setNarrationInterval(data.interval || 30);
      if (data.api_key_set && data.api_key_masked) {
        setApiKeyInput(data.api_key_masked);
      } else if (!data.api_key_set) {
        setApiKeyInput("");
      }
      if (data.models && data.models.length > 0) {
        setAvailableModels(data.models);
      }
      if (data.kokoro_enabled !== undefined) {
        setKokoroEnabled(data.kokoro_enabled || false);
      }
      if (data.kokoro_ip) {
        setKokoroIp(data.kokoro_ip);
      }
      if (data.kokoro_voice) {
        setKokoroVoice(data.kokoro_voice);
      }
      if (data.kokoro_voices && data.kokoro_voices.length > 0) {
        setAvailableKokoroVoices(data.kokoro_voices);
      }
      // Update ElevenLabs settings from narration config
      if (data.elevenlabs_api_key_set !== undefined) {
        setIsElevenLabsKeyValid(data.elevenlabs_api_key_set);
        if (!data.elevenlabs_api_key_set) {
          setElevenLabsApiKeyInput("");
        }
      }
      if (typeof data.elevenlabs_api_key === "string") {
        setElevenLabsApiKeyInput(data.elevenlabs_api_key);
      }
      if (data.elevenlabs_voice_id) {
        setSelectedElevenLabsVoice(data.elevenlabs_voice_id);
      }
    });

    socketClient.onKokoroValidationResult((data: KokoroValidationResult) => {
      console.log("🎤 SettingsDialog received Kokoro validation result:", data);
      setIsValidatingKokoro(false);
      if (data.valid && data.voices && data.voices.length > 0) {
        setAvailableKokoroVoices(data.voices);
        setKokoroError("");
        toast.success("🎤 Kokoro API Valid", {
          description: `${data.voices.length} voices available`,
          duration: 3000,
        });
        if (data.voices.length > 0 && !kokoroVoice) {
          setKokoroVoice(data.voices[0]);
          socketClient.emitKokoroConfigUpdate({
            kokoro_enabled: true,
            kokoro_ip: kokoroIp,
            kokoro_voice: data.voices[0],
          });
        }
      } else {
        setAvailableKokoroVoices([]);
        setKokoroError(data.error || "Failed to validate Kokoro API");
        toast.error("❌ Kokoro API Invalid", {
          description: data.error || "Could not connect to Kokoro API",
          duration: 4000,
        });
      }
    });

    socketClient.onKokoroConfigResponse((data) => {
      if (data.status === "ok") {
        toast.success("🎤 Kokoro Config Saved", { duration: 2000 });
      } else {
        toast.error("🎤 Kokoro Config Error", {
          description: data.message || "Failed to save Kokoro configuration",
          duration: 4000,
        });
      }
    });

    // Subscribe to ElevenLabs validation results
    socketClient.onElevenLabsValidationResult((data) => {
      console.log("🎤 SettingsDialog received ElevenLabs validation result:", data);
      setIsValidatingElevenLabs(false);
      if (data.valid && data.voices && data.voices.length > 0) {
        setAvailableElevenLabsVoices(data.voices);
        setIsElevenLabsKeyValid(true);
        setElevenLabsError("");
        
        const limitedMsg = data.error === 'limited_permissions' 
          ? " (using default voices due to limited API key permissions)" 
          : "";
        
        toast.success("🎤 ElevenLabs API Valid", {
          description: `${data.voices.length} voices available${limitedMsg}`,
          duration: 3000,
        });
        if (data.voices.length > 0) {
          setSelectedElevenLabsVoice(data.voices[0].voice_id);
          socketClient.emitStartupConfigUpdate({ elevenlabs_voice_id: data.voices[0].voice_id });
        }
      } else {
        setAvailableElevenLabsVoices([]);
        setIsElevenLabsKeyValid(false);
        setElevenLabsError(data.error || "Failed to validate ElevenLabs API");
        toast.error("❌ ElevenLabs API Invalid", {
          description: data.error || "Could not validate ElevenLabs API",
          duration: 4000,
        });
      }
    });

    // Subscribe to startup config sync
    socketClient.onStartupConfigSync((data) => {
      console.log("⬛ SettingsDialog received startup config sync:", data);
      setStartupCheckEnabled(data.startup_check_enabled);
      setIsElevenLabsKeyValid(data.elevenlabs_api_key_set);
      setSelectedElevenLabsVoice(data.elevenlabs_voice_id);
      if (typeof data.elevenlabs_api_key === "string") {
        setElevenLabsApiKeyInput(data.elevenlabs_api_key);
      } else if (!data.elevenlabs_api_key_set) {
        setElevenLabsApiKeyInput("");
      }
    });

    // Subscribe to startup check results
    socketClient.onStartupCheckResult((data) => {
      console.log("⬛ SettingsDialog received startup check result:", data);
      setIsRunningStartupCheck(false);
      if (data.status === "complete") {
        const allOk = data.all_ok ? "All systems operational" : "Some systems offline";
        const icon = data.critical_ok ? "✅" : "⚠️";
        toast.success(`${icon} Startup Check Complete`, {
          description: allOk,
          duration: 5000,
        });
      } else if (data.status === "error") {
        toast.error("⬛ Startup Check Failed", {
          description: data.error || "An error occurred during the check",
          duration: 4000,
        });
      }
    });

    // Request latest startup config after listeners are registered.
    // This avoids missing a fast sync response before callback binding.
    socketClient.requestStartupConfig();
  }, []);

  /* ---- AI Narration handlers ---- */
  const handleValidateKey = useCallback(() => {
    if (!apiKeyInput.trim()) {
      setKeyError("Please enter an API key");
      return;
    }
    if (apiKeyInput.startsWith("*")) {
      setKeyError("Key already saved. Enter a new key to re-validate.");
      return;
    }
    setIsValidatingKey(true);
    setKeyError("");
    socketClient.emitNarrationValidateKey(narrationProvider, apiKeyInput.trim());
  }, [apiKeyInput, narrationProvider]);

  const handleClearKey = useCallback(() => {
    socketClient.emitNarrationKeyClear();
    setApiKeyInput("");
    setIsKeyValid(false);
    setAvailableModels([]);
    setSelectedModel("");
    setKeyError("");
    toast.success("🗑️ API Key Cleared", { description: "API key removed from server", duration: 3000 });
  }, []);

  const handleModelChange = useCallback((modelName: string) => {
    setSelectedModel(modelName);
    socketClient.emitNarrationConfigUpdate({ model: modelName });
  }, []);

  const handleIntervalChange = useCallback((val: number) => {
    const clamped = Math.max(10, Math.min(300, val));
    setNarrationInterval(clamped);
    socketClient.emitNarrationConfigUpdate({ interval: clamped });
  }, []);

  /* ---- Kokoro handlers ---- */
  const handleKokoroValidateApi = useCallback(() => {
    if (!kokoroIp.trim()) {
      setKokoroError("Please enter a Kokoro API address");
      return;
    }
    setIsValidatingKokoro(true);
    setKokoroError("");
    socketClient.emitKokoroValidateApi(kokoroIp.trim());
  }, [kokoroIp]);

  const handleKokoroVoiceChange = useCallback(
    (voice: string) => {
      setKokoroVoice(voice);
      socketClient.emitKokoroConfigUpdate({ kokoro_enabled: kokoroEnabled, kokoro_ip: kokoroIp, kokoro_voice: voice });
    },
    [kokoroEnabled, kokoroIp],
  );

  const handleKokoroToggle = useCallback(
    (enabled: boolean) => {
      setKokoroEnabled(enabled);
      socketClient.emitKokoroConfigUpdate({ kokoro_enabled: enabled, kokoro_ip: kokoroIp, kokoro_voice: kokoroVoice });
    },
    [kokoroIp, kokoroVoice],
  );

  /* ---- ElevenLabs handlers ---- */
  const handleValidateElevenLabsKey = useCallback(() => {
    if (!elevenLabsApiKeyInput.trim()) {
      setElevenLabsError("Please enter an API key");
      return;
    }
    setIsValidatingElevenLabs(true);
    setElevenLabsError("");
    socketClient.emitElevenLabsValidateKey(elevenLabsApiKeyInput.trim());
  }, [elevenLabsApiKeyInput]);

  const handleClearElevenLabsKey = useCallback(() => {
    socketClient.emitElevenLabsKeyClear();
    setElevenLabsApiKeyInput("");
    setIsElevenLabsKeyValid(false);
    setAvailableElevenLabsVoices([]);
    setSelectedElevenLabsVoice("JBFqnCBsd6RMkjVDRZzb");
    setElevenLabsError("");
    toast.success("🗑️ ElevenLabs Key Cleared", { description: "API key removed from server", duration: 3000 });
  }, []);

  const handleElevenLabsVoiceChange = useCallback((voiceId: string) => {
    setSelectedElevenLabsVoice(voiceId);
    socketClient.emitStartupConfigUpdate({ elevenlabs_voice_id: voiceId });
  }, []);

  /* ---- Startup Check handlers ---- */
  const handleStartupCheckToggle = useCallback((enabled: boolean) => {
    setStartupCheckEnabled(enabled);
    socketClient.emitStartupConfigUpdate({ startup_check_enabled: enabled });
  }, []);

  const handleRunStartupCheck = useCallback(() => {
    setIsRunningStartupCheck(true);
    socketClient.emitStartupCheckRun();
  }, []);

  /* ---- CV mode camera helpers ---- */
  const originalCameraSettingsRef = useRef<{ resolution: string; jpeg_quality: number; framerate: number } | null>(null);

  const persistCameraConfig = useCallback(
    (config: { resolution: string; jpeg_quality: number; framerate: number; vision_enabled: boolean }) => {
      localStorage.setItem("cameraConfig", JSON.stringify(config));
    },
    [],
  );

  // Build dynamic tuning groups with resolution options from camera specs
  const getDynamicTuningGroups = (): typeof TUNING_GROUPS => {
    if (!cameraSpecs) return TUNING_GROUPS;
    const resolutionOptions = cameraSpecs.supported_modes.map((mode) => ({
      value: mode.toLowerCase().replace("x", "x"),
      label: mode,
    }));
    return TUNING_GROUPS.map((group) => {
      if (group.title === "CAMERA & VISION") {
        return {
          ...group,
          params: group.params.map((param) => (param.key === "CAMERA_RESOLUTION" ? { ...param, options: resolutionOptions } : param)),
        };
      }
      return group;
    });
  };

  const dynamicTuningGroups = getDynamicTuningGroups();

  const handleParamChange = useCallback(
    (key: keyof TuningConstants, value: number | string | boolean) => {
      if (key === "VISION_ENABLED" && value === true) {
        toast.error("CV Mode unavailable", {
          description: "MediaMTX-only camera mode does not support integrated vision analysis.",
          duration: 3500,
        });
        onTuningChange({ ...tuning, VISION_ENABLED: false });
        setSynced(false);
        return;
      }

      let updatedTuning = { ...tuning, [key]: value };

      // When VISION_ENABLED changes to true, enforce high-quality video settings
      if (key === "VISION_ENABLED" && value === true && tuning.VISION_ENABLED === false) {
        if (!originalCameraSettingsRef.current) {
          originalCameraSettingsRef.current = {
            resolution: tuning.CAMERA_RESOLUTION,
            jpeg_quality: tuning.CAMERA_JPEG_QUALITY,
            framerate: tuning.CAMERA_FRAMERATE,
          };
          console.log("📷 [CV Mode] Stored original camera settings:", originalCameraSettingsRef.current);
        }
        updatedTuning = {
          ...updatedTuning,
          CAMERA_RESOLUTION: CV_HIGH_QUALITY_SETTINGS.resolution,
          CAMERA_JPEG_QUALITY: CV_HIGH_QUALITY_SETTINGS.jpeg_quality,
          CAMERA_FRAMERATE: CV_HIGH_QUALITY_SETTINGS.framerate,
        };
        console.log(
          `📷 [CV Mode] Enabled – ${CV_HIGH_QUALITY_SETTINGS.resolution} @ ${CV_HIGH_QUALITY_SETTINGS.framerate}fps, ${CV_HIGH_QUALITY_SETTINGS.jpeg_quality}%`,
        );
        socketClient.emitCameraConfigUpdate(CV_HIGH_QUALITY_SETTINGS);
        persistCameraConfig({ ...CV_HIGH_QUALITY_SETTINGS, vision_enabled: true });
        socketClient.emitVisionToggle();
        toast.info("🎥 CV Mode Enabled", {
          description: `Video quality automatically set to ${CV_HIGH_QUALITY_SETTINGS.resolution} @ ${CV_HIGH_QUALITY_SETTINGS.framerate}fps, ${CV_HIGH_QUALITY_SETTINGS.jpeg_quality}% quality for optimal computer vision performance.`,
          duration: 4000,
        });
      }
      // When VISION_ENABLED changes to false, restore original settings
      else if (key === "VISION_ENABLED" && value === false && tuning.VISION_ENABLED === true) {
        const originalSettings = originalCameraSettingsRef.current || {
          resolution: tuning.CAMERA_RESOLUTION,
          jpeg_quality: tuning.CAMERA_JPEG_QUALITY,
          framerate: tuning.CAMERA_FRAMERATE,
        };
        const shouldRestore =
          tuning.CAMERA_RESOLUTION === CV_HIGH_QUALITY_SETTINGS.resolution &&
          tuning.CAMERA_JPEG_QUALITY === CV_HIGH_QUALITY_SETTINGS.jpeg_quality &&
          tuning.CAMERA_FRAMERATE === CV_HIGH_QUALITY_SETTINGS.framerate;
        if (shouldRestore && originalCameraSettingsRef.current) {
          updatedTuning = {
            ...updatedTuning,
            CAMERA_RESOLUTION: originalSettings.resolution,
            CAMERA_JPEG_QUALITY: originalSettings.jpeg_quality,
            CAMERA_FRAMERATE: originalSettings.framerate,
          };
          console.log("📷 [CV Mode] Disabled – Restoring original camera settings:", originalSettings);
          socketClient.emitCameraConfigUpdate(originalSettings);
          persistCameraConfig({ ...originalSettings, vision_enabled: false });
          toast.info("🎥 CV Mode Disabled", {
            description: "Video quality restored to previous settings.",
            duration: 3000,
          });
          originalCameraSettingsRef.current = null;
        } else {
          console.log("📷 [CV Mode] Disabled – Keeping current camera settings");
          persistCameraConfig({
            resolution: tuning.CAMERA_RESOLUTION,
            jpeg_quality: tuning.CAMERA_JPEG_QUALITY,
            framerate: tuning.CAMERA_FRAMERATE,
            vision_enabled: false,
          });
        }
        socketClient.emitVisionToggle();
      }

      onTuningChange(updatedTuning);
      setSynced(false);
    },
    [tuning, onTuningChange, persistCameraConfig],
  );

  const sendTuningToBackend = useCallback(
    (t: TuningConstants) => {
      const cameraConfig = { resolution: t.CAMERA_RESOLUTION, jpeg_quality: t.CAMERA_JPEG_QUALITY, framerate: t.CAMERA_FRAMERATE };
      console.log("📷 [Settings] Sending camera config to backend:", cameraConfig);
      socketClient.emitCameraConfigUpdate(cameraConfig);
      localStorage.setItem("cameraConfig", JSON.stringify({ ...cameraConfig, vision_enabled: t.VISION_ENABLED }));
      if (t.VISION_ENABLED !== tuning.VISION_ENABLED) socketClient.emitVisionToggle();
      const { CAMERA_RESOLUTION: _r, CAMERA_JPEG_QUALITY: _q, CAMERA_FRAMERATE: _f, VISION_ENABLED: _v, ...autopilotTuning } = t;
      socketClient.emitTuningUpdate(autopilotTuning);
      setSynced(true);
    },
    [tuning],
  );

  const handleApplyAndClose = useCallback(() => {
    sendTuningToBackend(tuning);
    setIsOpen(false);
  }, [tuning, sendTuningToBackend]);

  const handleResetDefaults = useCallback(() => {
    const defaults = { ...backendDefaults };
    onTuningChange(defaults);
    sendTuningToBackend(defaults);
  }, [backendDefaults, onTuningChange, sendTuningToBackend]);

  const handleResetDriver = useCallback(() => {
    if (confirm("Reset driver profile? You will be shown the onboarding screen again.")) {
      localStorage.removeItem('driverData');
      socketClient.emitResetDriverData();
      toast.success("Driver profile reset! 🔄");
      setIsOpen(false);
      if (onResetDriver) {
        onResetDriver();
      }
    }
  }, [onResetDriver]);

  return (
    <>
      {/* Settings Icon Button */}
      <button
        onClick={() => setIsOpen(true)}
        className="p-1.5 sm:p-2 rounded-lg border border-border bg-card text-muted-foreground hover:border-primary/50 hover:text-primary transition-all touch-feedback"
      >
        <Settings className="w-4 h-4 sm:w-5 sm:h-5" />
      </button>

      {/* Dialog Overlay – rendered via portal to escape stacking contexts */}
      {isOpen &&
        createPortal(
          <div className="fixed inset-0 z-[100] flex items-center justify-center bg-background/80 backdrop-blur-sm">
            <div className="racing-panel bg-card p-3 sm:p-4 w-[90vw] max-w-md max-h-[85vh] border border-primary/30 flex flex-col">
              {/* Header */}
              <div className="flex items-center justify-between mb-2 flex-shrink-0">
                <div className="flex items-center gap-2">
                  <Settings className="w-4 h-4 sm:w-5 sm:h-5 text-primary" />
                  <span className="racing-text text-xs sm:text-sm text-foreground">TUNING CONFIG</span>
                </div>
                <button onClick={() => setIsOpen(false)} className="p-1 rounded hover:bg-muted transition-colors">
                  <X className="w-4 h-4 text-muted-foreground" />
                </button>
              </div>

              {/* Scrollable Content */}
              <div className="flex-1 overflow-y-auto space-y-1.5 pr-1 min-h-0" data-scrollable="true" style={{ touchAction: "pan-y" }}>
                {/* ───────── SENSORS ───────── */}
                <CollapsibleGroup title="📡 SENSORS" defaultOpen={false}>
                  {[
                    { label: 'Front Sonar (HC-SR04)', enabled: isSonarEnabled, onToggle: onSonarToggle, required: true },
                    { label: 'Left IR', enabled: isIREnabled, onToggle: onIRToggle, required: true },
                    { label: 'Right IR', enabled: isIREnabled, onToggle: onIRToggle, required: true },
                    { label: 'MPU6050', enabled: isMPU6050Enabled, onToggle: onMPU6050Toggle, required: true },
                    { label: 'Camera', enabled: isCameraEnabled, onToggle: onCameraToggle, required: false },
                  ].map((sensor) => (
                    <div key={sensor.label} className="py-0.5">
                      <div className="flex items-center justify-between gap-1">
                        <div className="flex items-center gap-1 flex-1 min-w-0">
                          <div className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${sensor.enabled ? 'bg-emerald-400' : 'bg-gray-500'}`} />
                          <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text min-w-0 truncate">
                            {sensor.label}
                            {sensor.required && <span className="text-amber-400 ml-0.5">*</span>}
                          </span>
                        </div>
                        <button
                          onClick={sensor.onToggle}
                          disabled={isAutopilotRunning || (sensor.label === 'Left IR' || sensor.label === 'Right IR' ? false : false)}
                          className={`px-3 py-0.5 rounded border text-[10px] sm:text-xs racing-text transition-colors disabled:opacity-40 disabled:cursor-not-allowed ${
                            sensor.enabled
                              ? "border-primary bg-primary/20 text-primary hover:bg-primary/30"
                              : "border-border bg-muted/30 text-muted-foreground hover:bg-muted/50"
                          }`}
                        >
                          {sensor.enabled ? "ON" : "OFF"}
                        </button>
                      </div>
                    </div>
                  ))}
                  <div className="mt-1 px-1 py-0.5 text-[7px] sm:text-[8px] text-muted-foreground/60 racing-text">
                    <span className="text-amber-400">*</span> Required for Autopilot. Disabled sensors are ignored. Sensors cannot be toggled during autonomous driving.
                  </div>
                  {/* Note: Left IR and Right IR share a single toggle */}
                </CollapsibleGroup>

                {/* ───────── AI NARRATION ───────── */}
                <CollapsibleGroup title="🎙️ AI NARRATION" defaultOpen={false}>
                  {/* Provider */}
                  <SettingsRow label="Provider" info="AI provider used for image narration. Currently only Google Gemini is supported.">
                    <select
                      value={narrationProvider}
                      onChange={(e) => setNarrationProvider(e.target.value)}
                      className="px-2 py-0.5 rounded border border-border bg-card text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none"
                    >
                      <option value="gemini">Google Gemini</option>
                      <option value="openai" disabled>
                        OpenAI (Coming Soon)
                      </option>
                      <option value="anthropic" disabled>
                        Anthropic (Coming Soon)
                      </option>
                      <option value="ollama" disabled>
                        Ollama (Coming Soon)
                      </option>
                    </select>
                  </SettingsRow>

                  {/* API Key */}
                  <div className="py-0.5">
                    <div className="flex items-center justify-between gap-1">
                      <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text">API Key</span>
                      {isKeyValid && availableModels.length > 0 && (
                        <span className="flex items-center gap-0.5 text-[8px] text-green-400 racing-text">
                          <Check className="w-2.5 h-2.5" /> Cached ({availableModels.length} models)
                        </span>
                      )}
                      {isKeyValid && availableModels.length === 0 && (
                        <span className="flex items-center gap-0.5 text-[8px] text-green-400 racing-text">
                          <Check className="w-2.5 h-2.5" /> Saved
                        </span>
                      )}
                    </div>
                    <div className="flex items-center gap-0.5 mt-0.5">
                      <div className="relative flex-1">
                        <input
                          type={showApiKey ? "text" : "password"}
                          value={apiKeyInput}
                          onChange={(e) => {
                            const newValue = e.target.value;
                            setApiKeyInput(newValue);
                            // Only clear error if user is typing a new key
                            if (!isKeyValid || !newValue.startsWith("*")) {
                              setKeyError("");
                            }
                          }}
                          placeholder={isKeyValid && availableModels.length > 0 ? "✓ Key loaded from cache" : isKeyValid ? "•••••••• (key saved)" : "Paste your API key..."}
                          readOnly={isKeyValid && availableModels.length > 0}
                          className={`w-full h-5 sm:h-6 bg-card border border-border rounded px-2 pr-6 text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none placeholder:text-muted-foreground/50 ${
                            isKeyValid && availableModels.length > 0 ? "bg-muted/30 cursor-not-allowed" : ""
                          }`}
                        />
                        <button
                          onClick={() => setShowApiKey(!showApiKey)}
                          className="absolute right-1 top-1/2 -translate-y-1/2 text-muted-foreground/50 hover:text-primary/70"
                        >
                          {showApiKey ? <EyeOff className="w-3 h-3" /> : <Eye className="w-3 h-3" />}
                        </button>
                      </div>
                      {!(isKeyValid && availableModels.length > 0) && (
                        <button
                          onClick={handleValidateKey}
                          disabled={isValidatingKey || !apiKeyInput.trim() || apiKeyInput.startsWith("*")}
                          className="px-2 py-0.5 rounded border border-primary bg-primary/20 text-primary racing-text text-[9px] sm:text-[10px] hover:bg-primary/30 transition-colors disabled:opacity-40 disabled:pointer-events-none flex items-center gap-0.5"
                        >
                          {isValidatingKey ? (
                            <>
                              <Loader2 className="w-2.5 h-2.5 animate-spin" /> Checking...
                            </>
                          ) : (
                            "Validate"
                          )}
                        </button>
                      )}
                      {isKeyValid && (
                        <button
                          onClick={handleClearKey}
                          className="px-1.5 py-0.5 rounded border border-destructive/50 bg-destructive/10 text-destructive racing-text text-[9px] sm:text-[10px] hover:bg-destructive/20 transition-colors flex items-center gap-0.5"
                          title="Clear API key"
                        >
                          <Trash2 className="w-2.5 h-2.5" />
                        </button>
                      )}
                    </div>
                    {keyError && <div className="mt-0.5 text-[8px] text-red-400 racing-text">{keyError}</div>}
                    {isKeyValid && availableModels.length > 0 && (
                      <div className="mt-0.5 text-[7px] text-muted-foreground/60 racing-text">
                        ✓ API key loaded from config. Models cached locally.
                      </div>
                    )}
                  </div>

                  {/* Model */}
                  <SettingsRow label="Model" info="Multimodal (vision-capable) model used for image analysis.">
                    <select
                      value={selectedModel}
                      onChange={(e) => handleModelChange(e.target.value)}
                      disabled={!isKeyValid || availableModels.length === 0}
                      className="px-2 py-0.5 rounded border border-border bg-card text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none disabled:opacity-40 max-w-[180px]"
                    >
                      {availableModels.length === 0 ? (
                        <option value="">{isKeyValid ? "Loading models..." : "Validate key first"}</option>
                      ) : (
                        availableModels.map((m) => (
                          <option key={m.name} value={m.name}>
                            {m.display_name}
                          </option>
                        ))
                      )}
                    </select>
                  </SettingsRow>

                  {/* Interval */}
                  <SettingsRow label="Interval" info="Seconds between AI narration descriptions (10–300s).">
                    <div className="flex items-center gap-0.5">
                      <button
                        onClick={() => handleIntervalChange(narrationInterval - 1)}
                        disabled={narrationInterval <= 10}
                        className="w-5 h-5 sm:w-6 sm:h-6 rounded border border-border bg-muted flex items-center justify-center text-foreground hover:border-primary/50 hover:bg-primary/10 transition-colors touch-feedback disabled:opacity-30 disabled:pointer-events-none"
                      >
                        <Minus className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
                      </button>
                      <input
                        type="number"
                        value={narrationInterval}
                        onChange={(e) => handleIntervalChange(parseInt(e.target.value) || 30)}
                        min={10}
                        max={300}
                        className="w-12 sm:w-14 h-5 sm:h-6 bg-card border border-border rounded px-1 text-center text-[10px] sm:text-xs text-foreground racing-number focus:border-primary focus:outline-none [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
                      />
                      <button
                        onClick={() => handleIntervalChange(narrationInterval + 1)}
                        disabled={narrationInterval >= 300}
                        className="w-5 h-5 sm:w-6 sm:h-6 rounded border border-border bg-muted flex items-center justify-center text-foreground hover:border-primary/50 hover:bg-primary/10 transition-colors touch-feedback disabled:opacity-30 disabled:pointer-events-none"
                      >
                        <Plus className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
                      </button>
                      <span className="text-[8px] sm:text-[10px] text-muted-foreground racing-text w-4 text-right">s</span>
                    </div>
                  </SettingsRow>

                  {/* Analyse Image Toggle */}
                  <SettingsRow label="Analyse Image" info="When enabled, the AI will periodically capture and describe camera frames.">
                    <button
                      onClick={() => onImageAnalysisToggle?.(!imageAnalysisEnabled)}
                      className={`px-3 py-0.5 rounded border text-[10px] sm:text-xs racing-text transition-colors ${
                        imageAnalysisEnabled
                          ? "border-cyan-500 bg-cyan-500/20 text-cyan-400 hover:bg-cyan-500/30"
                          : "border-border bg-muted/30 text-muted-foreground hover:bg-muted/50"
                      }`}
                    >
                      {imageAnalysisEnabled ? "ON" : "OFF"}
                    </button>
                  </SettingsRow>
                  {imageAnalysisEnabled && (
                    <div className="ml-3 text-[7px] text-cyan-400/80 racing-text pb-0.5">
                      👁️ Image analysis active — AI will describe camera frames
                    </div>
                  )}

                  {/* AI Narration Toggle */}
                  <SettingsRow
                    label="AI Narration"
                    info="Enable spoken narration of AI image analysis via TTS. Requires a valid API key, model, and image analysis enabled."
                  >
                    <button
                      onClick={() => onImageAnalysisToggle?.(!imageAnalysisEnabled)}
                      disabled={!isKeyValid || !selectedModel || !imageAnalysisEnabled}
                      className={`px-3 py-0.5 rounded border text-[10px] sm:text-xs racing-text transition-colors flex items-center gap-1 disabled:opacity-40 disabled:pointer-events-none ${
                        imageAnalysisEnabled
                          ? "border-green-500 bg-green-500/20 text-green-400 hover:bg-green-500/30"
                          : "border-border bg-muted/30 text-muted-foreground hover:bg-muted/50"
                      }`}
                    >
                      {imageAnalysisEnabled ? (
                        <>
                          <Mic className="w-3 h-3" /> ON
                        </>
                      ) : (
                        <>
                          <MicOff className="w-3 h-3" /> OFF
                        </>
                      )}
                    </button>
                  </SettingsRow>
                  {!isKeyValid && (
                    <div className="ml-3 text-[7px] text-amber-400/80 racing-text pb-0.5">
                      ⚠️ Validate an API key and select a model to enable analysis
                    </div>
                  )}
                  {imageAnalysisEnabled && (
                    <div className="ml-3 text-[7px] text-green-400/80 racing-text pb-0.5">
                      🎙️ Analysis active — descriptions provided via TTS
                    </div>
                  )}
                </CollapsibleGroup>

                {/* ───────── KOKORO TTS ───────── */}
                <CollapsibleGroup title="🎤 KOKORO TTS" defaultOpen={false}>
                  {/* Enable / Disable */}
                  <SettingsRow label="Enable" info="Remote TTS server for enhanced speech synthesis (optional — local TTS fallback when disabled).">
                    <button
                      onClick={() => handleKokoroToggle(!kokoroEnabled)}
                      disabled={!availableKokoroVoices.length}
                      className={`px-3 py-0.5 rounded border text-[10px] sm:text-xs racing-text transition-colors disabled:opacity-40 disabled:pointer-events-none ${
                        kokoroEnabled
                          ? "border-purple-500 bg-purple-500/20 text-purple-400 hover:bg-purple-500/30"
                          : "border-border bg-muted/30 text-muted-foreground hover:bg-muted/50"
                      }`}
                    >
                      {kokoroEnabled ? "ON" : "OFF"}
                    </button>
                  </SettingsRow>

                  {/* API Address */}
                  <div className="py-0.5">
                    <div className="flex items-center justify-between gap-1">
                      <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text">API Address</span>
                      {availableKokoroVoices.length > 0 && (
                        <span className="flex items-center gap-0.5 text-[8px] text-green-400 racing-text">
                          <Check className="w-2.5 h-2.5" /> Connected
                        </span>
                      )}
                    </div>
                    <div className="flex items-center gap-0.5 mt-0.5">
                      <input
                        type="text"
                        value={kokoroIp}
                        onChange={(e) => {
                          setKokoroIp(e.target.value);
                          setKokoroError("");
                          setAvailableKokoroVoices([]);
                        }}
                        placeholder="192.169.29.105:8880"
                        className="flex-1 h-5 sm:h-6 bg-card border border-border rounded px-2 text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none placeholder:text-muted-foreground/50"
                      />
                      <button
                        onClick={handleKokoroValidateApi}
                        disabled={isValidatingKokoro || !kokoroIp.trim()}
                        className="px-2 py-0.5 rounded border border-purple-500 bg-purple-500/20 text-purple-400 racing-text text-[9px] sm:text-[10px] hover:bg-purple-500/30 transition-colors disabled:opacity-40 disabled:pointer-events-none flex items-center gap-0.5 whitespace-nowrap"
                      >
                        {isValidatingKokoro ? (
                          <>
                            <Loader2 className="w-2.5 h-2.5 animate-spin" /> Validating...
                          </>
                        ) : (
                          "Validate"
                        )}
                      </button>
                    </div>
                    {kokoroError && <div className="mt-0.5 text-[8px] text-red-400 racing-text">{kokoroError}</div>}
                  </div>

                  {/* Voice Selection */}
                  {availableKokoroVoices.length > 0 && (
                    <SettingsRow label="Voice" info="Select preferred voice for Kokoro TTS speech synthesis.">
                      <select
                        value={kokoroVoice}
                        onChange={(e) => handleKokoroVoiceChange(e.target.value)}
                        className="px-2 py-0.5 rounded border border-border bg-card text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none max-w-[180px]"
                      >
                        {availableKokoroVoices.map((voice) => (
                          <option key={voice} value={voice}>
                            {voice}
                          </option>
                        ))}
                      </select>
                    </SettingsRow>
                  )}
                </CollapsibleGroup>

                {/* ───────── ELEVENLABS TTS ───────── */}
                <CollapsibleGroup title="🎤 ELEVENLABS VOICES" defaultOpen={false}>
                  {/* Info */}
                  <div className="py-0.5 text-[8px] sm:text-[9px] text-muted-foreground/70 racing-text">
                    Configure premium AI voices for startup diagnostics and narration.
                  </div>

                  {/* API Key */}
                  <div className="py-0.5">
                    <div className="flex items-center justify-between gap-1">
                      <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text">API Key</span>
                      {isElevenLabsKeyValid && (
                        <span className="flex items-center gap-0.5 text-[8px] text-green-400 racing-text">
                          <Check className="w-2.5 h-2.5" /> Saved
                        </span>
                      )}
                    </div>
                    <div className="flex items-center gap-0.5 mt-0.5">
                      <div className="flex-1">
                        <input
                          type="text"
                          value={elevenLabsApiKeyInput}
                          onChange={(e) => {
                            setElevenLabsApiKeyInput(e.target.value);
                            setElevenLabsError("");
                          }}
                          placeholder="Paste your API key..."
                          className="w-full h-5 sm:h-6 bg-card border border-border rounded px-2 text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none placeholder:text-muted-foreground/50"
                        />
                      </div>
                      <button
                        onClick={handleValidateElevenLabsKey}
                        disabled={isValidatingElevenLabs || !elevenLabsApiKeyInput.trim()}
                        className="px-2 py-0.5 rounded border border-pink-500 bg-pink-500/20 text-pink-400 racing-text text-[9px] sm:text-[10px] hover:bg-pink-500/30 transition-colors disabled:opacity-40 disabled:pointer-events-none flex items-center gap-0.5"
                      >
                        {isValidatingElevenLabs ? (
                          <>
                            <Loader2 className="w-2.5 h-2.5 animate-spin" /> Checking...
                          </>
                        ) : (
                          "Validate"
                        )}
                      </button>
                      {isElevenLabsKeyValid && (
                        <button
                          onClick={handleClearElevenLabsKey}
                          className="px-1.5 py-0.5 rounded border border-destructive/50 bg-destructive/10 text-destructive racing-text text-[9px] sm:text-[10px] hover:bg-destructive/20 transition-colors flex items-center gap-0.5"
                          title="Clear API key"
                        >
                          <Trash2 className="w-2.5 h-2.5" />
                        </button>
                      )}
                    </div>
                    {elevenLabsError && <div className="mt-0.5 text-[8px] text-red-400 racing-text">{elevenLabsError}</div>}
                    {elevenLabsError && elevenLabsError.includes('permission') && (
                      <div className="mt-1 text-[7px] text-yellow-400/80 racing-text bg-yellow-500/10 p-1 rounded">
                        💡 To fix permission issues: Log into your ElevenLabs account → API Keys → Regenerate the key with full permissions including 'voices_read'
                      </div>
                    )}
                  </div>

                  {/* Voice Selection */}
                  <SettingsRow label="Default Voice" info="Voice used for startup AI diagnostics and narration.">
                    {availableElevenLabsVoices.length > 0 ? (
                      <select
                        value={selectedElevenLabsVoice}
                        onChange={(e) => handleElevenLabsVoiceChange(e.target.value)}
                        className="px-2 py-0.5 rounded border border-border bg-card text-[10px] sm:text-xs text-foreground racing-text focus:border-primary focus:outline-none max-w-[180px]"
                      >
                        {availableElevenLabsVoices.map((voice) => (
                          <option key={voice.voice_id} value={voice.voice_id}>
                            {voice.name}
                          </option>
                        ))}
                      </select>
                    ) : (
                      <input
                        type="text"
                        value={selectedElevenLabsVoice}
                        readOnly
                        className="w-full max-w-[220px] h-5 sm:h-6 bg-muted/30 border border-border rounded px-2 text-[10px] sm:text-xs text-foreground/80 racing-text cursor-not-allowed"
                      />
                    )}
                  </SettingsRow>
                </CollapsibleGroup>

                {/* ───────── STARTUP AI CHECK ───────── */}
                <CollapsibleGroup title="⬛ STARTUP AI CHECK" defaultOpen={false}>
                  {/* Enable / Disable */}
                  <SettingsRow label="Enable" info="Run AI diagnostics when the car starts up (speaks system status via TTS).">
                    <button
                      onClick={() => handleStartupCheckToggle(!startupCheckEnabled)}
                      className={`px-3 py-0.5 rounded border text-[10px] sm:text-xs racing-text transition-colors ${
                        startupCheckEnabled
                          ? "border-gray-500 bg-gray-500/20 text-gray-400 hover:bg-gray-500/30"
                          : "border-border bg-muted/30 text-muted-foreground hover:bg-muted/50"
                      }`}
                    >
                      {startupCheckEnabled ? "ON" : "OFF"}
                    </button>
                  </SettingsRow>

                  {/* Run Check Button */}
                  <div className="py-0.5">
                    <button
                      onClick={handleRunStartupCheck}
                      disabled={isRunningStartupCheck || !isElevenLabsKeyValid}
                      className="w-full px-3 py-1 rounded border border-gray-500 bg-gray-500/20 text-gray-400 racing-text text-[9px] sm:text-[10px] hover:bg-gray-500/30 transition-colors disabled:opacity-40 disabled:pointer-events-none flex items-center justify-center gap-1"
                    >
                      {isRunningStartupCheck ? (
                        <>
                          <Loader2 className="w-2.5 h-2.5 animate-spin" /> Running...
                        </>
                      ) : (
                        <>
                          ⬛ Run Startup Check
                        </>
                      )}
                    </button>
                  </div>

                  {!isElevenLabsKeyValid && (
                    <div className="ml-0 text-[7px] text-amber-400/80 racing-text pb-0.5">
                      ⚠️ Configure ElevenLabs API key above to enable startup checks
                    </div>
                  )}
                </CollapsibleGroup>

                {/* ───────── TUNING PARAMETER GROUPS ───────── */}
                {dynamicTuningGroups.map((group, i) => (
                  <CollapsibleGroup key={group.title} title={group.title} defaultOpen={i === 0}>
                    {group.params.map((param) => (
                      <ParamRow
                        key={param.key}
                        config={param}
                        value={tuning[param.key]}
                        onChange={(val) => handleParamChange(param.key, val)}
                        backendDefault={backendDefaults[param.key]}
                      />
                    ))}
                  </CollapsibleGroup>
                ))}
              </div>

              {/* Footer */}
              <div className="flex flex-col gap-2 mt-2 pt-2 border-t border-border/50 flex-shrink-0">
                <div className="flex gap-2">
                  <button
                    onClick={handleResetDefaults}
                    className="flex-1 py-1.5 px-3 rounded border border-border bg-muted/30 text-muted-foreground racing-text text-[10px] sm:text-xs hover:bg-muted/50 transition-colors touch-feedback"
                  >
                    RESET DEFAULTS
                  </button>
                  <button
                    onClick={handleApplyAndClose}
                    className="flex-1 py-1.5 px-3 rounded border border-primary bg-primary/20 text-primary racing-text text-[10px] sm:text-xs hover:bg-primary/30 transition-colors touch-feedback"
                  >
                    APPLY & CLOSE
                  </button>
                </div>
                <button
                  onClick={handleResetDriver}
                  className="w-full py-1.5 px-3 rounded border border-destructive/50 bg-destructive/10 text-destructive racing-text text-[10px] sm:text-xs hover:bg-destructive/20 transition-colors touch-feedback"
                >
                  RESET DRIVER PROFILE
                </button>
              </div>
            </div>
          </div>,
          document.body,
        )}
    </>
  );
};
