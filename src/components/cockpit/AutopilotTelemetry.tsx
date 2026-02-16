import { Navigation, AlertTriangle, RotateCcw, OctagonX, Compass, Activity, Gauge, Ruler, Play, Square, RefreshCw, Crosshair, ArrowRightLeft } from "lucide-react";

export type AutopilotStatus = 
  | "CRUISING"
  | "DODGING"
  | "ESCAPING"
  | "EMERGENCY_STOP";

interface AutopilotTelemetryProps {
  status: AutopilotStatus;
  accelerationPercent: number;
  distanceToObstacle: number; // in cm
  eBrakeActive?: boolean;
  isRunning?: boolean;
  // Slalom yaw-tracking telemetry
  targetYaw?: number;
  currentHeading?: number;
  slalomSign?: number;
  // MPU6050 Gyro telemetry
  gyroZ?: number;
  pidCorrection?: number;
  gyroAvailable?: boolean;
  gyroCalibrated?: boolean;
  isMPU6050Enabled?: boolean;
  onEmergencyStop?: () => void;
  onAutopilotToggle?: () => void;
  onStartStop?: () => void;
}

const STATUS_CONFIG: Record<AutopilotStatus, {
  icon: React.ElementType;
  label: string;
  color: string;
  bgColor: string;
  borderColor: string;
  description: string;
}> = {
  CRUISING: {
    icon: Navigation,
    label: "CRUISING",
    color: "text-primary",
    bgColor: "bg-primary/20",
    borderColor: "border-primary",
    description: "Gyro yaw-tracking active",
  },
  DODGING: {
    icon: ArrowRightLeft,
    label: "DODGING",
    color: "text-cyan-400",
    bgColor: "bg-cyan-400/20",
    borderColor: "border-cyan-400",
    description: "Slalom dodge — accumulating yaw",
  },
  ESCAPING: {
    icon: RotateCcw,
    label: "ESCAPING",
    color: "text-amber-500",
    bgColor: "bg-amber-500/20",
    borderColor: "border-amber-500",
    description: "Stop → reverse → spin escape",
  },
  EMERGENCY_STOP: {
    icon: OctagonX,
    label: "E-STOP",
    color: "text-destructive",
    bgColor: "bg-destructive/20",
    borderColor: "border-destructive",
    description: "Emergency — motors locked",
  },
};

// Fallback config for unknown states to prevent crashes
const FALLBACK_CONFIG = {
  icon: Activity,
  label: "UNKNOWN",
  color: "text-muted-foreground",
  bgColor: "bg-muted/20",
  borderColor: "border-muted-foreground",
  description: "Unknown state",
};

export const AutopilotTelemetry = ({
  status,
  accelerationPercent,
  distanceToObstacle,
  eBrakeActive = false,
  isRunning = false,
  targetYaw = 0,
  currentHeading = 0,
  slalomSign = 0,
  gyroZ = 0,
  pidCorrection = 0,
  gyroAvailable = false,
  gyroCalibrated = false,
  isMPU6050Enabled = true,
  onEmergencyStop,
  onAutopilotToggle,
  onStartStop,
}: AutopilotTelemetryProps) => {
  const config = STATUS_CONFIG[status] || FALLBACK_CONFIG;
  const StatusIcon = config.icon;
  
  // Determine distance warning level
  const distanceWarning = distanceToObstacle < 20 
    ? "text-destructive" 
    : distanceToObstacle < 50 
      ? "text-warning" 
      : "text-primary";

  return (
    <div className="flex flex-col items-center h-full py-0.5 px-1 gap-0.5">
      {/* Scrollable Content Area */}
      <div className="flex flex-col items-center flex-1 w-full overflow-y-auto gap-0.5 min-h-0">
        {/* Header */}
        <div className="flex items-center gap-1 flex-shrink-0">
          <Navigation className={`w-3 h-3 sm:w-4 sm:h-4 ${isRunning ? 'text-primary animate-pulse' : 'text-muted-foreground'}`} />
          <span className={`racing-text text-[8px] sm:text-xs font-bold ${isRunning ? 'text-primary' : 'text-muted-foreground'}`}>AUTOPILOT</span>
        </div>
      
      {/* Status Display */}
      <div className={`
        w-full p-1 rounded border-2 transition-all duration-300 flex-shrink-0
        flex flex-col items-center gap-0.5
        ${isRunning 
          ? `${config.borderColor} ${config.bgColor}` 
          : 'border-muted-foreground/30 bg-muted/10'}
      `}>
        <StatusIcon className={`w-5 h-5 sm:w-6 sm:h-6 ${
          isRunning 
            ? `${config.color} ${status === "ESCAPING" || status === "EMERGENCY_STOP" ? "animate-pulse" : ""}` 
            : 'text-muted-foreground'
        }`} />
        <span className={`racing-text text-[10px] sm:text-xs font-bold ${
          isRunning ? config.color : 'text-muted-foreground'
        }`}>
          {isRunning ? config.label : 'STANDBY'}
        </span>
        <span className="text-[6px] sm:text-[7px] text-muted-foreground text-center leading-tight">
          {isRunning ? config.description : 'Press start to begin'}
        </span>
      </div>
      
      {/* Acceleration Gauge */}
      <div className="w-full bg-card/50 rounded border border-border p-0.5 flex-shrink-0">
        <div className="flex items-center justify-between mb-0.5">
          <div className="flex items-center gap-0.5">
            <Gauge className="w-2.5 h-2.5 sm:w-3 sm:h-3 text-primary flex-shrink-0" />
            <span className="racing-text text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">ACCEL</span>
          </div>
          <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${accelerationPercent > 80 ? "text-primary text-glow-teal" : "text-foreground"}`}>
            {accelerationPercent.toFixed(0)}%
          </span>
        </div>
        <div className="w-full h-1 sm:h-1.5 bg-muted rounded-full overflow-hidden flex-shrink-0">
          <div 
            className="h-full bg-gradient-to-r from-primary/60 to-primary transition-all duration-200 rounded-full"
            style={{ width: `${accelerationPercent}%` }}
          />
        </div>
      </div>
      
      {/* Distance to Obstacle */}
      <div className="w-full bg-card/50 rounded border border-border p-0.5 flex-shrink-0">
        <div className="flex items-center justify-between mb-0.5">
          <div className="flex items-center gap-0.5">
            <Ruler className="w-2.5 h-2.5 sm:w-3 sm:h-3 text-cyan-500 flex-shrink-0" />
            <span className="racing-text text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">OBSTACLE</span>
          </div>
          <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${distanceWarning}`}>
            {distanceToObstacle}cm
          </span>
        </div>
        {/* Visual distance indicator */}
        <div className="w-full h-1 sm:h-1.5 bg-muted rounded-full overflow-hidden flex-shrink-0">
          <div 
            className={`h-full transition-all duration-200 rounded-full ${
              distanceToObstacle < 20 
                ? "bg-gradient-to-r from-destructive/60 to-destructive" 
                : distanceToObstacle < 50 
                  ? "bg-gradient-to-r from-warning/60 to-warning" 
                  : "bg-gradient-to-r from-cyan-500/60 to-cyan-500"
            }`}
            style={{ width: `${Math.min(100, (distanceToObstacle / 200) * 100)}%` }}
          />
        </div>
        <div className="flex justify-between mt-0.5">
          <span className="text-[5px] sm:text-[6px] text-destructive">NEAR</span>
          <span className="text-[5px] sm:text-[6px] text-success">FAR</span>
        </div>
      </div>
      
      {/* Slalom Yaw Heading */}
      <div className="w-full bg-card/50 rounded border border-border p-0.5 flex-shrink-0">
        <div className="flex items-center justify-between mb-0.5">
          <div className="flex items-center gap-0.5">
            <Compass className={`w-2.5 h-2.5 sm:w-3 sm:h-3 flex-shrink-0 ${isRunning ? 'text-cyan-400' : 'text-muted-foreground'}`} />
            <span className="racing-text text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">YAW</span>
          </div>
          <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${isRunning ? 'text-cyan-400' : 'text-muted-foreground'}`}>
            {isRunning ? `${currentHeading > 0 ? '+' : ''}${currentHeading.toFixed(0)}°` : 'OFF'}
          </span>
        </div>
        
        {isRunning ? (
          <>
            {/* Target vs Current heading */}
            <div className="flex items-center justify-between mb-0.5">
              <span className="text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">Target</span>
              <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${
                Math.abs(targetYaw) > 30 ? 'text-amber-400' : 'text-cyan-400'
              }`}>
                {targetYaw > 0 ? '+' : ''}{targetYaw.toFixed(0)}°
              </span>
            </div>
            <div className="flex items-center justify-between mb-0.5">
              <span className="text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">Dodge</span>
              <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${
                slalomSign > 0 ? 'text-cyan-400' : slalomSign < 0 ? 'text-amber-400' : 'text-muted-foreground'
              }`}>
                {slalomSign > 0 ? '→ RIGHT' : slalomSign < 0 ? '← LEFT' : '— NONE'}
              </span>
            </div>
            {/* Yaw heading bar — center = 0°, left/right = drift */}
            <div className="w-full h-1 sm:h-1.5 bg-muted rounded-full overflow-hidden relative flex-shrink-0">
              <div className="absolute left-1/2 top-0 w-px h-full bg-muted-foreground/40" />
              <div
                className={`absolute top-0 h-full rounded-full transition-all duration-200 ${
                  currentHeading >= 0
                    ? 'bg-gradient-to-r from-cyan-500/60 to-cyan-500'
                    : 'bg-gradient-to-l from-cyan-500/60 to-cyan-500'
                }`}
                style={{
                  left: currentHeading >= 0 ? '50%' : `${50 - Math.min(50, (Math.abs(currentHeading) / 60) * 50)}%`,
                  width: `${Math.min(50, (Math.abs(currentHeading) / 60) * 50)}%`,
                }}
              />
            </div>
            <div className="flex justify-between mt-0.5">
              <span className="text-[5px] sm:text-[6px] text-muted-foreground">← L</span>
              <span className="text-[5px] sm:text-[6px] text-muted-foreground">R →</span>
            </div>
          </>
        ) : (
          <div className="text-[6px] sm:text-[7px] text-muted-foreground text-center py-0.5">
            Yaw tracking activates on start
          </div>
        )}
      </div>
      
      {/* MPU6050 Gyro / PID Heading */}
      <div className="w-full bg-card/50 rounded border border-border p-0.5 flex-shrink-0">
        <div className="flex items-center justify-between mb-0.5">
          <div className="flex items-center gap-0.5">
            <Crosshair className={`w-2.5 h-2.5 sm:w-3 sm:h-3 flex-shrink-0 ${isMPU6050Enabled && gyroAvailable ? 'text-emerald-400' : 'text-muted-foreground'}`} />
            <span className="racing-text text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">GYRO</span>
          </div>
          <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${
            !isMPU6050Enabled ? 'text-muted-foreground'
            : !gyroAvailable ? 'text-destructive'
            : gyroCalibrated ? 'text-emerald-400'
            : 'text-amber-400'
          }`}>
            {!isMPU6050Enabled ? 'OFF' : !gyroAvailable ? 'N/A' : gyroCalibrated ? 'CAL' : 'UNCAL'}
          </span>
        </div>
        
        {isMPU6050Enabled && gyroAvailable && isRunning ? (
          <>
            {/* Yaw rate + PID correction */}
            <div className="flex items-center justify-between mb-0.5">
              <span className="text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">Yaw</span>
              <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${
                Math.abs(gyroZ) > 5 ? 'text-amber-400' : Math.abs(gyroZ) > 15 ? 'text-destructive' : 'text-emerald-400'
              }`}>
                {gyroZ > 0 ? '+' : ''}{gyroZ.toFixed(1)}°/s
              </span>
            </div>
            <div className="flex items-center justify-between mb-0.5">
              <span className="text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">PID</span>
              <span className={`racing-text text-[7px] sm:text-[8px] font-bold flex-shrink-0 ${
                Math.abs(pidCorrection) > 15 ? 'text-amber-400' : 'text-emerald-400'
              }`}>
                {pidCorrection > 0 ? '+' : ''}{pidCorrection.toFixed(1)}
              </span>
            </div>
            {/* Heading correction bar — center = straight, left/right = drift */}
            <div className="w-full h-1 sm:h-1.5 bg-muted rounded-full overflow-hidden relative flex-shrink-0">
              <div className="absolute left-1/2 top-0 w-px h-full bg-muted-foreground/40" />
              <div
                className={`absolute top-0 h-full rounded-full transition-all duration-200 ${
                  pidCorrection >= 0
                    ? 'bg-gradient-to-r from-emerald-500/60 to-emerald-500'
                    : 'bg-gradient-to-l from-emerald-500/60 to-emerald-500'
                }`}
                style={{
                  left: pidCorrection >= 0 ? '50%' : `${50 - Math.min(50, (Math.abs(pidCorrection) / 30) * 50)}%`,
                  width: `${Math.min(50, (Math.abs(pidCorrection) / 30) * 50)}%`,
                }}
              />
            </div>
            <div className="flex justify-between mt-0.5">
              <span className="text-[5px] sm:text-[6px] text-muted-foreground">← L</span>
              <span className="text-[5px] sm:text-[6px] text-muted-foreground">R →</span>
            </div>
          </>
        ) : (
          <div className="text-[6px] sm:text-[7px] text-muted-foreground text-center py-0.5">
            {!isMPU6050Enabled ? 'Gyroscope disabled' : !gyroAvailable ? 'MPU6050 not detected' : 'Active during cruise'}
          </div>
        )}
      </div>

      {/* Telemetry Wave - Active indicator */}
      <div className={`w-full overflow-hidden h-2 sm:h-3 border rounded flex-shrink-0 ${isRunning ? 'border-primary/50 bg-primary/10' : 'border-muted-foreground/30 bg-muted/20'}`}>
        {isRunning ? (
          <svg className="w-[200%] h-full animate-telemetry" viewBox="0 0 200 30" preserveAspectRatio="none">
            <path
              d="M0,15 Q10,5 20,15 T40,15 T60,15 T80,15 T100,15 T120,15 T140,15 T160,15 T180,15 T200,15"
              fill="none"
              stroke="hsl(var(--primary))"
              strokeWidth="2"
              className="opacity-90"
            />
            <path
              d="M0,15 Q10,25 20,15 T40,15 T60,15 T80,15 T100,15 T120,15 T140,15 T160,15 T180,15 T200,15"
              fill="none"
              stroke="hsl(var(--primary))"
              strokeWidth="1.5"
              className="opacity-60"
            />
          </svg>
        ) : (
          <svg className="w-full h-full" viewBox="0 0 200 30" preserveAspectRatio="none">
            <line x1="0" y1="15" x2="200" y2="15" stroke="hsl(var(--muted-foreground))" strokeWidth="1" className="opacity-30" />
          </svg>
        )}
      </div>
      <div className={`text-[5px] sm:text-[6px] racing-text font-bold flex-shrink-0 ${
        isRunning ? 'text-primary' : 'text-muted-foreground'
      }`}>
        {isRunning ? 'AUTOPILOT ACTIVE' : 'AUTOPILOT STANDBY'}
      </div>
      </div>
      
      {/* Control Buttons: E-BRAKE | EXIT | START/STOP - Always visible at bottom */}
      <div className="flex gap-1 w-full flex-shrink-0">
        {/* E-BRAKE Button */}
        <button
          onClick={onEmergencyStop}
          className={`
            flex-1 rounded-full border-2 flex items-center justify-center min-h-fit
            transition-all duration-100 touch-feedback font-bold racing-text
            ${eBrakeActive 
              ? 'bg-destructive border-destructive text-destructive-foreground hover:bg-destructive/90 shadow-lg shadow-destructive/50 animate-pulse' 
              : 'bg-card border-destructive/40 text-destructive/70 hover:bg-destructive/20 hover:border-destructive'}
          `}
        >
          <svg 
            className="w-8 h-8 sm:w-9 sm:h-9" 
            style={{ color: 'currentColor' }}
            xmlns="http://www.w3.org/2000/svg" 
            version="1.1" 
            viewBox="0 0 100 125"
          >
            <path d="M95.1,82.3l-41-3.7c-0.7-0.1-1.4-0.4-1.8-1L46,69.3l9.6-4.9l1.7,2.4c1.5,2.2,4.8,3,7,1.5c0.9-0.6,1.6-1.5,1.9-2.5    c1.4,0.5,3,0.3,4.3-0.6c0.9-0.6,1.5-1.4,1.8-2.4c1.4,0.5,3.1,0.3,4.4-0.6c1.4-0.9,2.1-2.4,2.1-4c1.5,0,2.9-0.6,4-1.9    c1.3-1.7,1.3-4.1,0.1-5.9l7.3-3.7c0.9-0.4,1.1-1.5,0.6-2.3l-0.5-0.8l1.9-1c0.9-0.4,1.1-1.5,0.6-2.3l-1.3-1.9    c-0.5-0.7-1.3-0.9-2.1-0.5L87.1,39L86.5,38c-0.5-0.7-1.3-0.9-2.1-0.5l-4.5,2.3l-2.8-5.5c-1.7-3.2-4.6-5.6-8.1-6.4l-8.7-2.2    l-2.2-3.1l0.7-0.5c1.1-0.8,1.4-2.3,0.6-3.4l-5.1-7.4c-0.8-1.1-2.3-1.4-3.4-0.6L32.4,23.5c-1.1,0.8-1.4,2.3-0.6,3.4l5.1,7.4    c0.8,1.1,2.3,1.4,3.4,0.6l0.7-0.5l2.2,3.2l-0.3,0.8c-1.6,3.6-1.2,7.8,1,11.1c1.3,1.9,2.9,4.1,4.8,6.4l-9.3,4.8l-2.5-3.3    c-0.8-1-2.1-1.3-3.2-0.8L4,71c-0.9,0.4-1.5,1.4-1.5,2.4v13.7c0,1.4,1.2,2.6,2.6,2.6h89.8c1.4,0,2.6-1.2,2.6-2.6V85    C97.5,83.6,96.5,82.5,95.1,82.3z M41.1,28.8c-1.1,0.7-2.5,0.5-3.3-0.6c-0.7-1.1-0.5-2.5,0.6-3.3c1.1-0.7,2.5-0.5,3.3,0.6    C42.4,26.6,42.1,28.1,41.1,28.8z M62.6,65.7c-0.8,0.6-2,0.4-2.6-0.5l-3-4.4c-0.6-0.8-0.4-2,0.5-2.6c0.9-0.6,2-0.4,2.6,0.5l3,4.4    C63.6,63.9,63.4,65.1,62.6,65.7z M68.7,62.6c-0.8,0.6-2,0.4-2.6-0.5l-4.1-6c-0.6-0.9-0.4-2,0.5-2.6c0.9-0.6,2-0.4,2.6,0.5l4.1,6    C69.8,60.8,69.6,62,68.7,62.6z M75,59.6c-0.8,0.6-2,0.4-2.6-0.5l-3.2-4.6c0.7-0.1,1.3-0.3,1.9-0.7l1.5-1l2.9,4.2    C76.1,57.9,75.9,59,75,59.6z M78.2,49l2.2,3.2c0.6,0.8,0.4,2-0.5,2.6c-0.8,0.6-2,0.4-2.6-0.5l-2.2-3.2l2.7-1.8    C78,49.2,78.1,49.1,78.2,49z M46.5,47.6c-1.6-2.4-1.9-5.4-0.7-8.1l0.7-1.6c0.2-0.5,0.2-1.1-0.2-1.5l-2.7-3.9l11.9-8.2L58,28    c0.2,0.3,0.5,0.5,0.9,0.6l9.3,2.3c2.6,0.7,4.9,2.4,6.1,4.8l3.1,6c0.9,1.7,0.3,3.9-1.3,4.9l-6.7,4.4c-0.8,0.5-1.9,0.3-2.5-0.5    c-0.3-0.4-0.4-0.9-0.3-1.4c0.1-0.5,0.4-0.9,0.8-1.2l4.7-3c0.7-0.4,0.9-1.3,0.6-2L71.1,40c-0.4-0.8-1.3-1.1-2.1-0.7    c-0.8,0.4-1.1,1.3-0.7,2.1l0.8,1.6c-6,1-8.7-3.6-8.8-3.9c-0.4-0.7-1.4-1-2.1-0.6L58,38.6c-0.6,0.4-0.9,1.3-0.5,2    c1,1.9,3.6,4.6,7.6,5.4c-0.7,0.6-1.1,1.3-1.4,2.2l-12.1,6.2C49.5,52,47.8,49.6,46.5,47.6z" fill="currentColor"/>
          </svg>
        </button>

        {/* Autopilot Exit Button */}
        <button
          onClick={onAutopilotToggle}
          className={`
            flex-1 rounded-full border-2 flex items-center justify-center min-h-fit
            transition-all duration-100 touch-feedback font-bold racing-text
            bg-card border-gold/60 text-gold hover:bg-gold/20 hover:border-gold
          `}
          title="Exit Autopilot Mode"
        >
          <svg
            className="w-8 h-8 sm:w-9 sm:h-9"
            style={{ color: 'currentColor' }}
            xmlns="http://www.w3.org/2000/svg"
            version="1.1"
            viewBox="-5.0 -10.0 110.0 135.0"
          >
            <path d="m50 81.25c17.234 0 31.25-14.016 31.25-31.25s-14.016-31.25-31.25-31.25-31.25 14.016-31.25 31.25 14.016 31.25 31.25 31.25zm0-59.375c15.516 0 28.125 12.609 28.125 28.125s-12.609 28.125-28.125 28.125-28.125-12.609-28.125-28.125 12.609-28.125 28.125-28.125z" fill="currentColor"/>
            <path d="m43.672 73.828s0.03125 0.015625 0.046875 0.015625h0.03125s0.09375 0.03125 0.15625 0.03125c1.9531 0.51562 4 0.78125 6.0938 0.78125s4.1406-0.26562 6.0938-0.78125c0.0625 0 0.10938-0.015625 0.15625-0.03125h0.03125s0.03125-0.015625 0.046875-0.015625c9.4062-2.5 16.625-10.438 18.062-20.219 0-0.0625 0.015625-0.10938 0.015625-0.17188 0.17188-1.125 0.25-2.2656 0.25-3.4375 0-2.4688-0.35938-4.875-1.0625-7.125 0-0.09375-0.046875-0.1875-0.078125-0.28125-3.1406-9.9844-12.5-17.25-23.516-17.25s-20.375 7.2656-23.516 17.25c-0.03125 0.09375-0.0625 0.1875-0.078125 0.28125-0.70312 2.25-1.0625 4.6562-1.0625 7.125 0 1.1719 0.078125 2.3125 0.25 3.4375 0 0.0625 0.015625 0.10938 0.015625 0.17188 1.4375 9.7812 8.6562 17.719 18.062 20.219zm1.4219-23.438c1.3281-1.3125 3.0625-2.0312 4.9062-2.0312s3.5781 0.71875 4.9062 2.0312c0.625 0.60938 0.625 1.5938 0.015625 2.2031-0.29688 0.3125-0.70312 0.46875-1.1094 0.46875s-0.79688-0.15625-1.0938-0.45312c-1.4688-1.4531-3.9688-1.4531-5.4375 0-0.60938 0.60938-1.6094 0.60938-2.2031-0.015625-0.60938-0.60938-0.60938-1.5938 0.015625-2.2031zm-1.625-1.5938c-0.60938 0.60938-1.5938 0.59375-2.2031-0.015625s-0.60938-1.5938 0-2.2031c2.3438-2.3281 5.4531-3.6094 8.7344-3.6094s6.3906 1.2812 8.7344 3.6094c0.60938 0.60938 0.60938 1.5938 0 2.2031-0.29688 0.3125-0.70312 0.46875-1.1094 0.46875s-0.79688-0.15625-1.0938-0.45312c-1.75-1.75-4.0781-2.7031-6.5312-2.7031s-4.7812 0.95312-6.5312 2.7031zm5 6.6719c0-0.85938 0.6875-1.5625 1.5469-1.5625h0.015625c0.85938 0 1.5625 0.70312 1.5625 1.5625s-0.70312 1.5625-1.5625 1.5625-1.5625-0.70312-1.5625-1.5625zm8.6875 14.828c-0.0625-0.54688-0.09375-1.0781-0.09375-1.6094 0-7.625 6.2031-13.828 13.828-13.828h0.078125c-1.6562 7.2031-6.9375 13.016-13.812 15.438zm-7.1562-41.828c8.4688 0 15.797 4.9062 19.328 12.031-6.2344-2.0312-12.719-3.0625-19.328-3.0625s-13.094 1.0312-19.328 3.0625c3.5312-7.125 10.859-12.031 19.328-12.031zm-20.891 26.391c7.625 0 13.828 6.2031 13.828 13.828 0 0.53125-0.03125 1.0625-0.09375 1.6094-6.875-2.4219-12.156-8.2344-13.812-15.438z" fill="currentColor"/>
            <path d="m22.375 22.375c14.375-14.375 37.234-15.172 52.562-2.4375l-2.9375 0.1875c-0.85938 0.046875-1.5156 0.79688-1.4688 1.6562 0.046875 0.82812 0.73438 1.4688 1.5625 1.4688h0.09375l6.6094-0.42188c0.078125 0 0.14062-0.046875 0.21875-0.0625 0.10938-0.015625 0.21875-0.046875 0.32812-0.09375s0.1875-0.125 0.28125-0.1875c0.0625-0.046875 0.125-0.0625 0.1875-0.125 0 0 0-0.03125 0.03125-0.046875 0.078125-0.078125 0.125-0.1875 0.1875-0.28125 0.046875-0.078125 0.10938-0.14062 0.14062-0.23438 0.03125-0.078125 0.03125-0.17188 0.046875-0.26562 0.015625-0.10938 0.046875-0.21875 0.046875-0.34375v-0.046875l-0.42188-6.6094c-0.0625-0.85938-0.78125-1.5156-1.6562-1.4688-0.85938 0.046875-1.5156 0.79688-1.4688 1.6562l0.17188 2.7656c-16.531-13.703-41.203-12.828-56.719 2.6875-10.25 10.25-14.484 24.844-11.328 39.016 0.15625 0.73438 0.8125 1.2188 1.5312 1.2188 0.10938 0 0.23438 0 0.34375-0.03125 0.84375-0.1875 1.375-1.0156 1.1875-1.8594-2.9219-13.125 1-26.625 10.5-36.125z" fill="currentColor"/>
            <path d="m91.172 40.828c-0.1875-0.84375-1.0156-1.375-1.8594-1.1875s-1.375 1.0156-1.1875 1.8594c2.9219 13.125-1 26.625-10.5 36.125-14.375 14.375-37.234 15.172-52.562 2.4375l2.9375-0.1875c0.85938-0.046875 1.5156-0.79688 1.4688-1.6562s-0.78125-1.5-1.6562-1.4688l-6.6094 0.42188c-0.078125 0-0.14062 0.046875-0.21875 0.0625-0.10938 0.015625-0.21875 0.046875-0.32812 0.09375s-0.1875 0.125-0.28125 0.1875c-0.0625 0.046875-0.125 0.0625-0.1875 0.125 0 0 0 0.03125-0.03125 0.046875-0.078125 0.078125-0.125 0.1875-0.1875 0.28125-0.046875 0.078125-0.10938 0.14062-0.14062 0.23438-0.03125 0.078125-0.03125 0.17188-0.046875 0.26562-0.015625 0.10938-0.046875 0.21875-0.046875 0.34375v0.046875l0.42188 6.6094c0.046875 0.82812 0.73438 1.4688 1.5625 1.4688h0.09375c0.85938-0.046875 1.5156-0.79688 1.4688-1.6562l-0.17188-2.7656c7.7812 6.4531 17.344 9.7031 26.922 9.7031 10.797 0 21.609-4.1094 29.828-12.344 10.25-10.25 14.484-24.844 11.328-39.016z" fill="currentColor"/>
          </svg>
        </button>

        {/* START / STOP Button */}
        <button
          onClick={onStartStop}
          disabled={eBrakeActive}
          className={`
            flex-1 rounded-full border-2 flex items-center justify-center min-h-fit
            transition-all duration-100 touch-feedback font-bold racing-text
            ${eBrakeActive
              ? 'bg-card border-muted-foreground/30 text-muted-foreground cursor-not-allowed opacity-50'
              : isRunning
                ? 'bg-card border-amber-500/60 text-amber-500 hover:bg-amber-500/20 hover:border-amber-500 shadow-md shadow-amber-500/20'
                : 'bg-card border-emerald-500/60 text-emerald-500 hover:bg-emerald-500/20 hover:border-emerald-500 shadow-md shadow-emerald-500/20'
            }
          `}
          title={isRunning ? 'Stop Autopilot' : 'Start Autopilot'}
        >
          {isRunning ? (
            <Square className="w-7 h-7 sm:w-8 sm:h-8" />
          ) : (
            <Play className="w-7 h-7 sm:w-8 sm:h-8" />
          )}
        </button>
      </div>
    </div>
  );
};
