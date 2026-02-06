import { useState, useCallback } from "react";

interface PedalsProps {
  onThrottleChange: (active: boolean) => void;
  onBrakeChange: (active: boolean) => void;
}

export const Pedals = ({ onThrottleChange, onBrakeChange }: PedalsProps) => {
  const [throttlePressed, setThrottlePressed] = useState(false);
  const [brakePressed, setBrakePressed] = useState(false);

  const handleThrottleStart = useCallback((e: React.TouchEvent | React.MouseEvent) => {
    e.preventDefault();
    setThrottlePressed(true);
    onThrottleChange(true);
  }, [onThrottleChange]);

  const handleThrottleEnd = useCallback(() => {
    setThrottlePressed(false);
    onThrottleChange(false);
  }, [onThrottleChange]);

  const handleBrakeStart = useCallback((e: React.TouchEvent | React.MouseEvent) => {
    e.preventDefault();
    setBrakePressed(true);
    onBrakeChange(true);
  }, [onBrakeChange]);

  const handleBrakeEnd = useCallback(() => {
    setBrakePressed(false);
    onBrakeChange(false);
  }, [onBrakeChange]);

  return (
    <div className="flex h-full gap-0.5 p-0.5">
      {/* Brake Pedal - 30% width */}
      <button
        className={`
          flex-[0.3] h-full rounded-lg border-2 flex flex-col items-center justify-center
          transition-all duration-75 select-none touch-none
          ${brakePressed 
            ? 'bg-gradient-to-t from-destructive to-destructive/70 border-destructive glow-red scale-[0.98]' 
            : 'bg-gradient-to-t from-destructive/40 to-destructive/20 border-destructive/50 hover:border-destructive/70'
          }
        `}
        onTouchStart={handleBrakeStart}
        onTouchEnd={handleBrakeEnd}
        onTouchCancel={handleBrakeEnd}
        onMouseDown={handleBrakeStart}
        onMouseUp={handleBrakeEnd}
        onMouseLeave={handleBrakeEnd}
      >
        <div className={`racing-text text-xs sm:text-lg font-bold transition-all ${brakePressed ? 'text-white text-glow-red' : 'text-destructive-foreground/70'}`}>
          BRAKE
        </div>
        <div className="flex gap-0.5 mt-0.5">
          {[...Array(3)].map((_, i) => (
            <div 
              key={i}
              className={`w-1 sm:w-1.5 h-0.5 rounded-full transition-colors ${brakePressed ? 'bg-white' : 'bg-destructive/50'}`}
            />
          ))}
        </div>
      </button>

      {/* Throttle Pedal - 70% width */}
      <button
        className={`
          flex-[0.7] h-full rounded-lg border-2 flex flex-col items-center justify-center
          transition-all duration-75 select-none touch-none
          ${throttlePressed 
            ? 'bg-gradient-to-t from-primary to-primary/70 border-primary glow-teal scale-[0.98]' 
            : 'bg-gradient-to-t from-primary/40 to-primary/20 border-primary/50 hover:border-primary/70'
          }
        `}
        onTouchStart={handleThrottleStart}
        onTouchEnd={handleThrottleEnd}
        onTouchCancel={handleThrottleEnd}
        onMouseDown={handleThrottleStart}
        onMouseUp={handleThrottleEnd}
        onMouseLeave={handleThrottleEnd}
      >
        <div className={`racing-text text-sm sm:text-xl font-bold transition-all ${throttlePressed ? 'text-white text-glow-teal' : 'text-primary-foreground/70'}`}>
          THROTTLE
        </div>
        <div className="flex gap-0.5 mt-0.5">
          {[...Array(5)].map((_, i) => (
            <div 
              key={i}
              className={`w-1.5 sm:w-2 h-0.5 sm:h-1 rounded-full transition-all ${
                throttlePressed 
                  ? 'bg-white' 
                  : 'bg-primary/50'
              }`}
              style={{ 
                transitionDelay: throttlePressed ? `${i * 30}ms` : '0ms',
                opacity: throttlePressed ? 1 : 0.5
              }}
            />
          ))}
        </div>
        
        {/* Power Meter */}
        <div className="w-3/4 h-1 sm:h-1.5 bg-card/50 rounded-full mt-1 overflow-hidden">
          <div 
            className={`h-full bg-gradient-to-r from-primary via-primary-glow to-white rounded-full transition-all duration-100 ${throttlePressed ? 'w-full' : 'w-0'}`}
          />
        </div>
      </button>
    </div>
  );
};
