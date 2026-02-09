import { useState, useCallback } from "react";
import { createPortal } from "react-dom";
import { Settings, X, Minus, Plus, ChevronDown, ChevronRight } from "lucide-react";

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
}

const TUNING_GROUPS: { title: string; params: ParamConfig[] }[] = [
  {
    title: "DISTANCE THRESHOLDS",
    params: [
      { key: "FRONT_CRITICAL_CM", label: "Front Critical", min: 2, max: 100, step: 1, unit: "cm" },
      { key: "REAR_BLOCKED_CM", label: "Rear Blocked", min: 2, max: 100, step: 1, unit: "cm" },
      { key: "REAR_CRITICAL_CM", label: "Rear Critical", min: 2, max: 100, step: 1, unit: "cm" },
      { key: "DANGER_CM", label: "Danger Zone", min: 2, max: 100, step: 1, unit: "cm" },
      { key: "FULL_SPEED_CM", label: "Full Speed", min: 100, max: 500, step: 5, unit: "cm" },
    ],
  },
  {
    title: "SPEED PROFILES",
    params: [
      { key: "MAX_SPEED", label: "Max Cruise", min: 10, max: 100, step: 5, unit: "%" },
      { key: "MIN_SPEED", label: "Min Cruise", min: 10, max: 100, step: 5, unit: "%" },
      { key: "REVERSE_SPEED", label: "Reverse", min: 10, max: 100, step: 5, unit: "%" },
      { key: "PIVOT_SPEED", label: "Pivot", min: 10, max: 100, step: 5, unit: "%" },
    ],
  },
  {
    title: "TIMING",
    params: [
      { key: "REVERSE_DURATION", label: "Reverse Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
      { key: "REVERSE_STEP", label: "Reverse Step", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
      { key: "PIVOT_DURATION", label: "Pivot Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
      { key: "RECOVERY_DURATION", label: "Recovery Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
      { key: "STUCK_RECHECK_INTERVAL", label: "Stuck Recheck", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
    ],
  },
  {
    title: "FILTER",
    params: [
      { key: "SONAR_HISTORY_LEN", label: "Sonar Filter Window", min: 1, max: 5, step: 1, unit: "" },
    ],
  },
  {
    title: "SMART POWER",
    params: [
      { key: "STUCK_DISTANCE_THRESH", label: "Stuck Distance", min: 1, max: 20, step: 1, unit: "cm" },
      { key: "STUCK_TIME_THRESH", label: "Stuck Time", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
      { key: "STUCK_BOOST_STEP", label: "Boost Step", min: 1, max: 20, step: 1, unit: "%" },
      { key: "STUCK_BOOST_MAX", label: "Boost Max", min: 30, max: 100, step: 5, unit: "%" },
      { key: "STUCK_MOVE_RESET", label: "Move Reset", min: 1, max: 20, step: 1, unit: "cm" },
    ],
  },
  {
    title: "ESCALATING ESCAPE",
    params: [
      { key: "MAX_NORMAL_ESCAPES", label: "Max Escapes", min: 1, max: 10, step: 1, unit: "" },
      { key: "UTURN_SPEED", label: "U-Turn Speed", min: 20, max: 100, step: 5, unit: "%" },
      { key: "UTURN_DURATION", label: "U-Turn Duration", min: 0.1, max: 5.0, step: 0.1, unit: "s" },
      { key: "ESCAPE_CLEAR_CM", label: "Escape Clear", min: 5, max: 100, step: 5, unit: "cm" },
    ],
  },
];

interface SettingsDialogProps {
  tuning: TuningConstants;
  onTuningChange: (tuning: TuningConstants) => void;
}

const ParamRow = ({
  config,
  value,
  onChange,
}: {
  config: ParamConfig;
  value: number;
  onChange: (val: number) => void;
}) => {
  const clamp = (v: number) => Math.min(config.max, Math.max(config.min, v));
  const decimals = config.step < 1 ? 1 : 0;

  const handleManualChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const raw = e.target.value;
    if (raw === "" || raw === "-") return;
    const parsed = parseFloat(raw);
    if (!isNaN(parsed)) {
      onChange(clamp(parsed));
    }
  };

  return (
    <div className="flex items-center justify-between gap-1 py-0.5">
      <span className="text-[9px] sm:text-[11px] text-muted-foreground racing-text flex-1 min-w-0 truncate">
        {config.label}
      </span>
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

export const SettingsDialog = ({ tuning, onTuningChange }: SettingsDialogProps) => {
  const [isOpen, setIsOpen] = useState(false);

  const handleParamChange = useCallback(
    (key: keyof TuningConstants, value: number) => {
      onTuningChange({ ...tuning, [key]: value });
    },
    [tuning, onTuningChange]
  );

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
                    />
                  ))}
                </CollapsibleGroup>
              ))}
            </div>

            {/* Footer */}
            <div className="flex gap-2 mt-2 pt-2 border-t border-border/50 flex-shrink-0">
              <button
                onClick={() => onTuningChange({ ...DEFAULT_TUNING })}
                className="flex-1 py-1.5 px-3 rounded border border-border bg-muted/30 text-muted-foreground racing-text text-[10px] sm:text-xs hover:bg-muted/50 transition-colors touch-feedback"
              >
                RESET DEFAULTS
              </button>
              <button
                onClick={() => setIsOpen(false)}
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
