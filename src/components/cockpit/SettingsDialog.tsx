import { useState, useCallback } from "react";
import { createPortal } from "react-dom";
import { Settings, X, Minus, Plus, ChevronDown, ChevronRight, HelpCircle } from "lucide-react";
import * as socketClient from "../../lib/socketClient";

export interface TuningConstants {
  // Tuning constants
  FRONT_CRITICAL_CM: number;
  REAR_BLOCKED_CM: number;
  REAR_CRITICAL_CM: number;
  DANGER_CM: number;
  FULL_SPEED_CM: number;
  MAX_SPEED: number;
  MIN_SPEED: number;
  REVERSE_SPEED: number;
  PIVOT_SPEED: number;
  REVERSE_DURATION: number;
  REVERSE_STEP: number;
  PIVOT_DURATION: number;
  RECOVERY_DURATION: number;
  STUCK_RECHECK_INTERVAL: number;
  SONAR_HISTORY_LEN: number;
  // Smart Power Management
  STUCK_DISTANCE_THRESH: number;
  STUCK_TIME_THRESH: number;
  STUCK_BOOST_STEP: number;
  STUCK_BOOST_MAX: number;
  STUCK_MOVE_RESET: number;
  // Escalating Escape
  MAX_NORMAL_ESCAPES: number;
  UTURN_SPEED: number;
  UTURN_DURATION: number;
  ESCAPE_CLEAR_CM: number;
}

export const DEFAULT_TUNING: TuningConstants = {
  FRONT_CRITICAL_CM: 5,
  REAR_BLOCKED_CM: 3,
  REAR_CRITICAL_CM: 5,
  DANGER_CM: 40,
  FULL_SPEED_CM: 100,
  MAX_SPEED: 80,
  MIN_SPEED: 30,
  REVERSE_SPEED: 40,
  PIVOT_SPEED: 50,
  REVERSE_DURATION: 1,
  REVERSE_STEP: 1.0,
  PIVOT_DURATION: 1.0,
  RECOVERY_DURATION: 1.0,
  STUCK_RECHECK_INTERVAL: 1.0,
  SONAR_HISTORY_LEN: 3,
  STUCK_DISTANCE_THRESH: 2,
  STUCK_TIME_THRESH: 1.0,
  STUCK_BOOST_STEP: 5,
  STUCK_BOOST_MAX: 80,
  STUCK_MOVE_RESET: 5,
  MAX_NORMAL_ESCAPES: 2,
  UTURN_SPEED: 70,
  UTURN_DURATION: 0.8,
  ESCAPE_CLEAR_CM: 20,
};

interface ParamConfig {
  key: keyof TuningConstants;
  label: string;
  min: number;
  max: number;
  step: number;
  unit: string;
  info: string;
}

const TUNING_GROUPS: { title: string; params: ParamConfig[] }[] = [
  {
    title: "DISTANCE THRESHOLDS",
    params: [
      { key: "FRONT_CRITICAL_CM", label: "Front Critical", min: 2, max: 100, step: 1, unit: "cm",
        info: "Front sonar distance that triggers an emergency escape maneuver. ↑ Increase: reacts earlier, more cautious. ↓ Decrease: gets closer before braking, riskier." },
      { key: "REAR_BLOCKED_CM", label: "Rear Blocked", min: 2, max: 100, step: 1, unit: "cm",
        info: "Minimum rear clearance required to reverse. Below this the car refuses to back up. ↑ Increase: needs more space behind to reverse. ↓ Decrease: allows reversing in tighter spaces." },
      { key: "REAR_CRITICAL_CM", label: "Rear Critical", min: 2, max: 100, step: 1, unit: "cm",
        info: "Rear sonar distance that interrupts an in-progress reverse. ↑ Increase: stops reversing sooner. ↓ Decrease: reverses closer to rear obstacles." },
      { key: "DANGER_CM", label: "Danger Zone", min: 2, max: 100, step: 1, unit: "cm",
        info: "Distance threshold that triggers deceleration and escape logic. ↑ Increase: begins slowing from further away. ↓ Decrease: higher speed closer to obstacles." },
      { key: "FULL_SPEED_CM", label: "Full Speed", min: 100, max: 500, step: 5, unit: "cm",
        info: "Distance above which the car cruises at maximum speed. ↑ Increase: needs more open space for full throttle. ↓ Decrease: reaches max speed sooner." },
    ],
  },
  {
    title: "SPEED PROFILES",
    params: [
      { key: "MAX_SPEED", label: "Max Cruise", min: 10, max: 100, step: 5, unit: "%",
        info: "Maximum PWM duty cycle during open-road cruising. ↑ Increase: faster top speed. ↓ Decrease: slower, more controlled driving." },
      { key: "MIN_SPEED", label: "Min Cruise", min: 10, max: 100, step: 5, unit: "%",
        info: "Minimum PWM at the danger zone boundary. The car never goes slower than this while moving. ↑ Increase: faster in tight spaces. ↓ Decrease: creeps more carefully near walls." },
      { key: "REVERSE_SPEED", label: "Reverse", min: 10, max: 100, step: 5, unit: "%",
        info: "PWM duty cycle while reversing during escape maneuvers. ↑ Increase: reverses faster. ↓ Decrease: slower, more controlled reverse." },
      { key: "PIVOT_SPEED", label: "Pivot", min: 10, max: 100, step: 5, unit: "%",
        info: "PWM duty cycle during pivot/tank turns. ↑ Increase: snappier turns. ↓ Decrease: gentler rotation." },
    ],
  },
  {
    title: "TIMING",
    params: [
      { key: "REVERSE_DURATION", label: "Reverse Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "Total time allowed for reverse maneuver. ↑ Increase: reverses further back. ↓ Decrease: shorter reverse distance." },
      { key: "REVERSE_STEP", label: "Reverse Step", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "Duration of each reverse micro-step before re-checking rear sensors. ↑ Increase: longer bursts between checks. ↓ Decrease: more frequent safety checks." },
      { key: "PIVOT_DURATION", label: "Pivot Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "How long the car pivots to change direction. ↑ Increase: larger turn angle. ↓ Decrease: smaller adjustments." },
      { key: "RECOVERY_DURATION", label: "Recovery Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "Pause after maneuvers to let sensors stabilize. ↑ Increase: more stable but slower recovery. ↓ Decrease: faster resumption, may get noisy readings." },
      { key: "STUCK_RECHECK_INTERVAL", label: "Stuck Recheck", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "Time between checks while stuck. ↑ Increase: waits longer between attempts. ↓ Decrease: retries more frequently." },
    ],
  },
  {
    title: "FILTER",
    params: [
      { key: "SONAR_HISTORY_LEN", label: "Sonar Filter Window", min: 1, max: 5, step: 1, unit: "",
        info: "Median filter window size for sonar readings. ↑ Increase: smoother but slower response. ↓ Decrease: more responsive but noisier." },
    ],
  },
  {
    title: "SMART POWER",
    params: [
      { key: "STUCK_DISTANCE_THRESH", label: "Stuck Distance", min: 1, max: 20, step: 1, unit: "cm",
        info: "Distance change below which the car is considered 'not moving'. ↑ Increase: harder to detect as stuck. ↓ Decrease: triggers stuck detection more easily." },
      { key: "STUCK_TIME_THRESH", label: "Stuck Time", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "Seconds of no movement before power boost kicks in. ↑ Increase: waits longer before boosting. ↓ Decrease: boosts sooner when stuck." },
      { key: "STUCK_BOOST_STEP", label: "Boost Step", min: 1, max: 20, step: 1, unit: "%",
        info: "PWM percentage added each stuck interval. ↑ Increase: more aggressive power ramp. ↓ Decrease: gentler power increases." },
      { key: "STUCK_BOOST_MAX", label: "Boost Max", min: 30, max: 100, step: 5, unit: "%",
        info: "Absolute maximum PWM cap with boost applied. ↑ Increase: allows higher max power. ↓ Decrease: limits boost ceiling." },
      { key: "STUCK_MOVE_RESET", label: "Move Reset", min: 1, max: 20, step: 1, unit: "cm",
        info: "Distance change that confirms the car is moving again, resetting boost. ↑ Increase: needs more movement to clear stuck state. ↓ Decrease: resets boost sooner." },
    ],
  },
  {
    title: "ESCALATING ESCAPE",
    params: [
      { key: "MAX_NORMAL_ESCAPES", label: "Max Escapes", min: 1, max: 10, step: 1, unit: "",
        info: "Normal escape attempts before triggering a U-turn. ↑ Increase: tries more before escalating. ↓ Decrease: escalates to U-turn sooner." },
      { key: "UTURN_SPEED", label: "U-Turn Speed", min: 20, max: 100, step: 5, unit: "%",
        info: "PWM during 180° spin escape. ↑ Increase: faster spin. ↓ Decrease: slower, more controlled U-turn." },
      { key: "UTURN_DURATION", label: "U-Turn Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s",
        info: "How long the U-turn spin lasts. ↑ Increase: wider turn arc. ↓ Decrease: smaller rotation." },
      { key: "ESCAPE_CLEAR_CM", label: "Escape Clear", min: 5, max: 100, step: 5, unit: "cm",
        info: "Distance that confirms a clear path, resetting the escape counter. ↑ Increase: needs more space to reset. ↓ Decrease: resets counter sooner." },
    ],
  },
];

interface SettingsDialogProps {
  tuning: TuningConstants;
  onTuningChange: (tuning: TuningConstants) => void;
  backendDefaults?: TuningConstants;
}

const ParamRow = ({
  config,
  value,
  onChange,
  backendDefault,
}: {
  config: ParamConfig;
  value: number;
  onChange: (val: number) => void;
  backendDefault: number;
}) => {
  const [showInfo, setShowInfo] = useState(false);
  const clamp = (v: number) => Math.min(config.max, Math.max(config.min, v));
  const decimals = config.step < 1 ? 1 : 0;
  const defaultVal = backendDefault;

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
        <div className="flex items-center gap-0.5 flex-1 min-w-0">
          <button
            onClick={() => setShowInfo(!showInfo)}
            className={`flex-shrink-0 p-0 transition-colors ${showInfo ? 'text-primary' : 'text-muted-foreground/50 hover:text-primary/70'}`}
          >
            <HelpCircle className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
          </button>
          <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text min-w-0 truncate">
            {config.label}
          </span>
        </div>
        <div className="flex items-center gap-0.5">
          <button
            onClick={() => onChange(clamp(+(value - config.step).toFixed(2)))}
            disabled={value <= config.min}
            className="w-5 h-5 sm:w-6 sm:h-6 rounded border border-border bg-muted flex items-center justify-center text-foreground hover:border-primary/50 hover:bg-primary/10 transition-colors touch-feedback disabled:opacity-30 disabled:pointer-events-none"
          >
            <Minus className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
          </button>
          <input
            type="number"
            value={Number(value.toFixed(decimals))}
            onChange={handleManualChange}
            min={config.min}
            max={config.max}
            step={config.step}
            className="w-12 sm:w-14 h-5 sm:h-6 bg-card border border-border rounded px-1 text-center text-[10px] sm:text-xs text-foreground racing-number focus:border-primary focus:outline-none [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
          />
          <button
            onClick={() => onChange(clamp(+(value + config.step).toFixed(2)))}
            disabled={value >= config.max}
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
        <div className="mt-0.5 ml-3 p-1.5 rounded bg-primary/5 border border-primary/20 text-[7px] sm:text-[9px] text-muted-foreground leading-relaxed">
          {config.info}
          <div className="mt-0.5 text-primary/70 font-semibold">
            DEFAULT: {defaultVal}{config.unit ? ` ${config.unit}` : ''} · RANGE: {config.min}–{config.max}
          </div>
        </div>
      )}
    </div>
  );
};

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

export const SettingsDialog = ({ tuning, onTuningChange, backendDefaults = DEFAULT_TUNING }: SettingsDialogProps) => {
  const [isOpen, setIsOpen] = useState(false);
  const [synced, setSynced] = useState(false);

  const handleParamChange = useCallback(
    (key: keyof TuningConstants, value: number) => {
      onTuningChange({ ...tuning, [key]: value });
      setSynced(false);
    },
    [tuning, onTuningChange]
  );

  const sendTuningToBackend = useCallback((t: TuningConstants) => {
    socketClient.emitTuningUpdate(t as unknown as Record<string, number>);
    setSynced(true);
  }, []);

  const handleApplyAndClose = useCallback(() => {
    sendTuningToBackend(tuning);
    setIsOpen(false);
  }, [tuning, sendTuningToBackend]);

  const handleResetDefaults = useCallback(() => {
    const defaults = { ...backendDefaults };
    onTuningChange(defaults);
    sendTuningToBackend(defaults);
  }, [backendDefaults, onTuningChange, sendTuningToBackend]);

  return (
    <>
      {/* Settings Icon Button */}
      <button
        onClick={() => setIsOpen(true)}
        className="p-1.5 sm:p-2 rounded-lg border border-border bg-card text-muted-foreground hover:border-primary/50 hover:text-primary transition-all touch-feedback"
      >
        <Settings className="w-4 h-4 sm:w-5 sm:h-5" />
      </button>

      {/* Dialog Overlay - rendered via portal to escape Header's stacking context */}
      {isOpen && createPortal(
        <div className="fixed inset-0 z-[9999] flex items-center justify-center bg-background/80 backdrop-blur-sm">
          <div className="racing-panel bg-card p-3 sm:p-4 w-[90vw] max-w-md max-h-[85vh] border border-primary/30 flex flex-col">
            {/* Header */}
            <div className="flex items-center justify-between mb-2 flex-shrink-0">
              <div className="flex items-center gap-2">
                <Settings className="w-4 h-4 sm:w-5 sm:h-5 text-primary" />
                <span className="racing-text text-xs sm:text-sm text-foreground">TUNING CONFIG</span>
              </div>
              <button
                onClick={() => setIsOpen(false)}
                className="p-1 rounded hover:bg-muted transition-colors"
              >
                <X className="w-4 h-4 text-muted-foreground" />
              </button>
            </div>

            {/* Scrollable Content */}
            <div className="flex-1 overflow-y-auto space-y-1.5 pr-1 min-h-0">
              {TUNING_GROUPS.map((group, i) => (
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
            <div className="flex gap-2 mt-2 pt-2 border-t border-border/50 flex-shrink-0">
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
          </div>
        </div>,
        document.body
      )}
    </>
  );
};
