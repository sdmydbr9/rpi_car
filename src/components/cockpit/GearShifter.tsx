import { OctagonX, Zap, Power, PowerOff, Radio, Eye, Radar, Plane, X, Navigation } from "lucide-react";

interface GearShifterProps {
  currentGear: string;
  onGearChange: (gear: string) => void;
  isEmergencyStop: boolean;
  isAutoMode: boolean;
  isIREnabled: boolean;
  isSonarEnabled: boolean;
  isAutopilotEnabled: boolean;
  onEmergencyStop: () => void;
  onAutoMode: () => void;
  onIRToggle: () => void;
  onSonarToggle: () => void;
  onAutopilotToggle: () => void;
  isEnabled?: boolean;
  isEngineRunning?: boolean;
  onEngineStart?: () => void;
  onEngineStop?: () => void;
}

const GEARS = ["S", "3", "2", "1", "N", "R"];

export const GearShifter = ({ 
  currentGear, 
  onGearChange,
  isEmergencyStop,
  isAutoMode,
  isIREnabled,
  isSonarEnabled,
  isAutopilotEnabled,
  onEmergencyStop,
  onAutoMode,
  onIRToggle,
  onSonarToggle,
  onAutopilotToggle,
  isEnabled = true,
  isEngineRunning = false,
  onEngineStart,
  onEngineStop
}: GearShifterProps) => {
  // Gearbox layout: 2 rows x 3 columns
  const gearLayout = [
    ["S", "3", "2"],
    ["1", "N", "R"]
  ];

  return (
    <div className="flex flex-col items-center h-full pt-1 pb-1 px-1.5 overflow-hidden bg-gradient-to-b from-background to-background/80">
      {/* LIVE TELEMETRY Header */}
      <div className="text-[7px] sm:text-[8px] font-bold racing-text text-muted-foreground tracking-wider mb-1.5">
        LIVE TELEMETRY
      </div>

      {/* E-STOP and AUTO OFF - Side by Side Pill Shaped */}
      <div className="flex w-full gap-1 mb-1.5">
        {/* E-STOP Button - Pill Shaped */}
        <button
          onClick={onEmergencyStop}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-1.5 px-2
            transition-all duration-100 touch-feedback font-bold racing-text
            ${isEmergencyStop
              ? 'bg-destructive border-destructive text-destructive-foreground shadow-lg shadow-destructive/50'
              : 'bg-card border-destructive/60 text-destructive hover:bg-destructive/20 hover:border-destructive'
            }
          `}
        >
          <X className="w-3.5 h-3.5 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">E-STOP</span>
        </button>

        {/* AUTO OFF Button - Pill Shaped */}
        <button
          onClick={onAutoMode}
          disabled={isEmergencyStop}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-1.5 px-2
            transition-all duration-100 touch-feedback font-bold racing-text disabled:opacity-50
            ${isAutoMode
              ? 'bg-primary border-primary text-primary-foreground shadow-lg shadow-primary/50'
              : 'bg-card border-primary/60 text-primary hover:bg-primary/20 hover:border-primary'
            }
          `}
        >
          <Zap className="w-3.5 h-3.5 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">AUTO {isAutoMode ? "ON" : "OFF"}</span>
        </button>
      </div>

      {/* IR and SONAR and AUTOPILOT - Side by Side Round Buttons */}
      <div className="flex w-full gap-1 mb-1.5 justify-center">
        {/* IR Button - Round */}
        <button
          onClick={onIRToggle}
          disabled={!isEngineRunning}
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center
            transition-all duration-100 touch-feedback font-bold
            ${!isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : isIREnabled
              ? 'bg-amber-500/30 border-amber-500 text-amber-500 shadow-lg'
              : 'bg-card border-amber-500/40 text-amber-500/60 hover:bg-amber-500/20 hover:border-amber-500'
            }
          `}
          title={isIREnabled ? 'IR: ON' : 'IR: OFF'}
        >
          <Eye className="w-5 h-5" />
        </button>

        {/* SONAR Button - Round */}
        <button
          onClick={onSonarToggle}
          disabled={!isEngineRunning}
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center
            transition-all duration-100 touch-feedback font-bold
            ${!isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : isSonarEnabled
              ? 'bg-primary border-primary text-primary-foreground shadow-lg'
              : 'bg-card border-primary/60 text-primary hover:bg-primary/20 hover:border-primary'
            }
          `}
          title={isSonarEnabled ? 'SONAR: ON' : 'SONAR: OFF'}
        >
          <Radar className="w-5 h-5" />
        </button>

        {/* AUTOPILOT - Round Button */}
        <button
          onClick={onAutopilotToggle}
          disabled={!isEngineRunning || isEmergencyStop}
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center
            transition-all duration-100 touch-feedback font-bold
            ${!isEngineRunning || isEmergencyStop
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : isAutopilotEnabled
              ? 'bg-primary border-primary text-primary-foreground shadow-lg'
              : 'bg-card border-primary/60 text-primary hover:bg-primary/20 hover:border-primary'
            }
          `}
          title={isAutopilotEnabled ? 'AUTOPILOT: ON' : 'AUTOPILOT: OFF'}
        >
          <Navigation className="w-5 h-5" />
        </button>
      </div>

      {/* Gearbox Grid Container */}
      <div className="bg-card border-2 border-border rounded-lg p-1.5 space-y-1 mb-1.5">
        {gearLayout.map((row, rowIdx) => (
          <div key={rowIdx} className="flex gap-1 justify-center">
            {row.map((gear) => {
              const isActive = currentGear === gear;
              const isReverse = gear === "R";
              
              return (
                <button
                  key={gear}
                  onClick={() => onGearChange(gear)}
                  disabled={!isEnabled}
                  className={`
                    w-7 h-7 sm:w-8 sm:h-8 rounded-lg border-2 text-[10px] sm:text-xs font-bold racing-text
                    transition-all duration-150 touch-feedback flex items-center justify-center
                    ${!isEnabled
                      ? 'bg-muted/40 border-muted/30 text-muted-foreground opacity-50 cursor-not-allowed'
                      : isActive
                      ? isReverse
                        ? "gear-reverse-active border-destructive bg-destructive/20"
                        : "gear-active border-primary bg-primary/20"
                      : "bg-card border-border hover:border-primary/60 text-muted-foreground hover:text-foreground hover:bg-primary/5"
                    }
                  `}
                >
                  {gear}
                </button>
              );
            })}
          </div>
        ))}
      </div>

      {/* Spacer */}
      <div className="flex-1" />

      {/* START and STOP Buttons */}
      <div className="flex gap-1.5 w-full">
        {/* START Button - Pill Shaped */}
        <button
          onClick={onEngineStart}
          disabled={isEngineRunning}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-2 px-1.5
            transition-all duration-100 touch-feedback font-bold racing-text
            ${isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : 'bg-primary border-primary text-primary-foreground hover:bg-primary/90 shadow-lg'
            }
          `}
        >
          <Power className="w-4 h-4 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">START</span>
        </button>

        {/* STOP Button - Pill Shaped */}
        <button
          onClick={onEngineStop}
          disabled={!isEngineRunning}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-2 px-1.5
            transition-all duration-100 touch-feedback font-bold racing-text
            ${!isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : 'bg-destructive border-destructive text-destructive-foreground hover:bg-destructive/90 shadow-lg'
            }
          `}
        >
          <PowerOff className="w-4 h-4 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">STOP</span>
        </button>
      </div>
    </div>
  );
};