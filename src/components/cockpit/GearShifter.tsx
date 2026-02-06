import { OctagonX, Zap, Power, PowerOff, Radio } from "lucide-react";

interface GearShifterProps {
  currentGear: string;
  onGearChange: (gear: string) => void;
  isEmergencyStop: boolean;
  isAutoMode: boolean;
  isIREnabled: boolean;
  onEmergencyStop: () => void;
  onAutoMode: () => void;
  onIRToggle: () => void;
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
  onEmergencyStop,
  onAutoMode,
  onIRToggle,
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
    <div className="flex flex-col items-center h-full pt-1 pb-1 px-1 overflow-hidden">
      <div className="racing-text text-[8px] sm:text-xs text-muted-foreground mb-2">TRANSMISSION</div>
      
      {/* Gearbox Grid Container */}
      <div className="bg-card border-2 border-border rounded-lg p-2 space-y-1.5">
        {gearLayout.map((row, rowIdx) => (
          <div key={rowIdx} className="flex gap-1.5 justify-center">
            {row.map((gear) => {
              const isActive = currentGear === gear;
              const isReverse = gear === "R";
              
              return (
                <button
                  key={gear}
                  onClick={() => onGearChange(gear)}
                  disabled={!isEnabled}
                  className={`
                    w-10 h-10 sm:w-12 sm:h-12 rounded-lg border-2 text-xs sm:text-sm font-bold racing-text
                    transition-all duration-150 touch-feedback flex items-center justify-center
                    ${!isEnabled
                      ? 'bg-muted/40 border-muted/30 text-muted-foreground opacity-50 cursor-not-allowed'
                      : isActive
                      ? isReverse
                        ? "gear-reverse-active border-destructive bg-destructive/20 shadow-lg shadow-destructive/50"
                        : "gear-active border-primary bg-primary/20 shadow-lg shadow-primary/50"
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
      
      {/* Telemetry Wave */}
      <div className="w-full mt-2 overflow-hidden h-3 sm:h-4 border border-border rounded bg-card/50">
        <svg className="w-[200%] h-full animate-telemetry" viewBox="0 0 200 30" preserveAspectRatio="none">
          <path
            d="M0,15 Q10,5 20,15 T40,15 T60,15 T80,15 T100,15 T120,15 T140,15 T160,15 T180,15 T200,15"
            fill="none"
            stroke="hsl(var(--primary))"
            strokeWidth="1.5"
            className="opacity-70"
          />
          <path
            d="M0,15 Q10,25 20,15 T40,15 T60,15 T80,15 T100,15 T120,15 T140,15 T160,15 T180,15 T200,15"
            fill="none"
            stroke="hsl(var(--primary))"
            strokeWidth="1"
            className="opacity-40"
          />
        </svg>
      </div>
      <div className="text-[5px] sm:text-[7px] text-muted-foreground racing-text mt-0.5 mb-2">TELEMETRY</div>
      
      {/* Emergency Brake and AUTO Mode Buttons */}
      <div className="flex gap-1 mt-1 w-full px-1">
        {/* Emergency Brake Button */}
        <button
          onClick={onEmergencyStop}
          className={`
            flex-1 rounded-md border-2 flex flex-col items-center justify-center p-1 h-8
            transition-all duration-100 touch-feedback
            ${isEmergencyStop
              ? 'bg-destructive border-destructive text-destructive-foreground glow-red'
              : 'bg-card border-destructive/50 text-destructive hover:bg-destructive/20'
            }
          `}
        >
          <OctagonX className="w-3 h-3 sm:w-3.5 sm:h-3.5" />
          <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-tight">BRAKE</span>
        </button>
        
        {/* AUTO Button */}
        <button
          onClick={onAutoMode}
          disabled={isEmergencyStop}
          className={`
            flex-1 rounded-md border-2 flex flex-col items-center justify-center p-1 h-8
            transition-all duration-100 touch-feedback disabled:opacity-50
            ${isAutoMode
              ? 'bg-primary border-primary text-primary-foreground glow-teal'
              : 'bg-card border-primary/50 text-primary hover:bg-primary/20'
            }
          `}
        >
          <Zap className="w-3 h-3 sm:w-3.5 sm:h-3.5" />
          <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-tight">AUTO</span>
        </button>
      </div>

      {/* START and STOP Buttons */}
      <div className="flex gap-1 mt-1 w-full px-1">
        {/* START Button */}
        <button
          onClick={onEngineStart}
          disabled={isEngineRunning}
          className={`
            flex-1 rounded-md border-2 flex flex-col items-center justify-center p-1 h-8
            transition-all duration-100 touch-feedback
            ${isEngineRunning
              ? 'bg-muted border-muted/50 text-muted-foreground cursor-not-allowed opacity-50'
              : 'bg-primary border-primary text-primary-foreground hover:bg-primary/90 glow-teal'
            }
          `}
        >
          <Power className="w-3 h-3 sm:w-3.5 sm:h-3.5" />
          <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-tight">START</span>
        </button>

        {/* STOP Button */}
        <button
          onClick={onEngineStop}
          disabled={!isEngineRunning}
          className={`
            flex-1 rounded-md border-2 flex flex-col items-center justify-center p-1 h-8
            transition-all duration-100 touch-feedback
            ${!isEngineRunning
              ? 'bg-muted border-muted/50 text-muted-foreground cursor-not-allowed opacity-50'
              : 'bg-destructive border-destructive text-destructive-foreground hover:bg-destructive/90 glow-red'
            }
          `}
        >
          <PowerOff className="w-3 h-3 sm:w-3.5 sm:h-3.5" />
          <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-tight">STOP</span>
        </button>
      </div>

      {/* IR Sensor Button */}
      <div className="flex gap-1 mt-1 w-full px-1">
        <button
          onClick={onIRToggle}
          disabled={!isEngineRunning}
          className={`
            w-12 h-12 mx-auto rounded-full border-2 flex items-center justify-center
            transition-all duration-100 touch-feedback
            ${!isEngineRunning
              ? 'bg-muted border-muted/50 text-muted-foreground cursor-not-allowed opacity-50'
              : isIREnabled
              ? 'bg-primary border-primary text-primary-foreground glow-teal'
              : 'bg-card border-primary/50 text-primary hover:bg-primary/20'
            }
          `}
          title={isIREnabled ? 'IR Sensors: ON' : 'IR Sensors: OFF'}
        >
          <Radio className="w-4 h-4 sm:w-5 sm:h-5" />
        </button>
        <div className="flex-1" />
      </div>
    </div>
  );
};