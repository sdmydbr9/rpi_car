import { useState, useCallback, useRef } from "react";
import { useTouchTracking } from "@/hooks/useTouchTracking";

interface PedalsProps {
  onThrottleChange: (active: boolean) => void;
  onBrakeChange: (active: boolean) => void;
  isEnabled?: boolean;
}

export const Pedals = ({ onThrottleChange, onBrakeChange, isEnabled = true }: PedalsProps) => {
  const [throttlePressed, setThrottlePressed] = useState(false);
  const [brakePressed, setBrakePressed] = useState(false);

  const throttleRef = useRef<HTMLButtonElement>(null);
  const brakeRef = useRef<HTMLButtonElement>(null);

  // --- Multitouch-safe: each pedal tracks its own finger independently ---
  // This allows holding throttle with one finger while tapping brake with another.

  useTouchTracking(throttleRef, {
    onTouchStart: () => {
      setThrottlePressed(true);
      onThrottleChange(true);
      navigator.vibrate?.(15);
    },
    onTouchEnd: () => {
      setThrottlePressed(false);
      onThrottleChange(false);
    },
  }, isEnabled);

  useTouchTracking(brakeRef, {
    onTouchStart: () => {
      setBrakePressed(true);
      onBrakeChange(true);
      navigator.vibrate?.(15);
    },
    onTouchEnd: () => {
      setBrakePressed(false);
      onBrakeChange(false);
    },
  }, isEnabled);

  // Mouse handlers for desktop (no multitouch issue with mouse)
  const handleThrottleMouseDown = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    setThrottlePressed(true);
    onThrottleChange(true);
  }, [onThrottleChange]);

  const handleThrottleMouseUp = useCallback(() => {
    setThrottlePressed(false);
    onThrottleChange(false);
  }, [onThrottleChange]);

  const handleBrakeMouseDown = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    setBrakePressed(true);
    onBrakeChange(true);
  }, [onBrakeChange]);

  const handleBrakeMouseUp = useCallback(() => {
    setBrakePressed(false);
    onBrakeChange(false);
  }, [onBrakeChange]);

  return (
    <div className="flex h-full gap-0.5 p-0.5">
      {/* Brake Pedal - 30% width */}
      <button
        ref={brakeRef}
        disabled={!isEnabled}
        className={`
          flex-[0.3] h-full rounded-lg border-2 flex flex-col items-center justify-center
          transition-all duration-75 select-none touch-none
          ${!isEnabled
            ? 'bg-muted/40 border-muted/30 opacity-50 cursor-not-allowed'
            : brakePressed 
            ? 'bg-gradient-to-t from-destructive to-destructive/70 border-destructive glow-red scale-[0.98]' 
            : 'bg-gradient-to-t from-destructive/40 to-destructive/20 border-destructive/50 hover:border-destructive/70'
          }
        `}
        onMouseDown={!isEnabled ? undefined : handleBrakeMouseDown}
        onMouseUp={!isEnabled ? undefined : handleBrakeMouseUp}
        onMouseLeave={!isEnabled ? undefined : handleBrakeMouseUp}
      >
        <div className={`racing-text text-xs sm:text-lg font-bold transition-all ${!isEnabled ? 'text-muted-foreground' : brakePressed ? 'text-white text-glow-red' : 'text-destructive-foreground/70'}`}>
          BRAKE
        </div>
        <div className="flex gap-0.5 mt-0.5">
          {[...Array(3)].map((_, i) => (
            <div 
              key={i}
              className={`w-1 sm:w-1.5 h-0.5 rounded-full transition-colors ${!isEnabled ? 'bg-muted/30' : brakePressed ? 'bg-white' : 'bg-destructive/50'}`}
            />
          ))}
        </div>
      </button>

      {/* Throttle Pedal - 70% width */}
      <button
        ref={throttleRef}
        disabled={!isEnabled}
        className={`
          flex-[0.7] h-full rounded-lg border-2 flex flex-col items-center justify-center
          transition-all duration-75 select-none touch-none
          ${!isEnabled
            ? 'bg-muted/40 border-muted/30 opacity-50 cursor-not-allowed'
            : throttlePressed 
            ? 'bg-gradient-to-t from-primary to-primary/70 border-primary glow-teal scale-[0.98]' 
            : 'bg-gradient-to-t from-primary/40 to-primary/20 border-primary/50 hover:border-primary/70'
          }
        `}
        onMouseDown={!isEnabled ? undefined : handleThrottleMouseDown}
        onMouseUp={!isEnabled ? undefined : handleThrottleMouseUp}
        onMouseLeave={!isEnabled ? undefined : handleThrottleMouseUp}
      >
        <div className={`racing-text text-sm sm:text-xl font-bold transition-all ${!isEnabled ? 'text-muted-foreground' : throttlePressed ? 'text-white text-glow-teal' : 'text-primary-foreground/70'}`}>
          THROTTLE
        </div>
        <div className="flex gap-0.5 mt-0.5">
          {[...Array(5)].map((_, i) => (
            <div 
              key={i}
              className={`w-1.5 sm:w-2 h-0.5 sm:h-1 rounded-full transition-all ${
                !isEnabled
                  ? 'bg-muted/30'
                  : throttlePressed 
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
            className={`h-full bg-gradient-to-r from-primary via-primary-glow to-white rounded-full transition-all duration-100 ${!isEnabled ? 'w-0 bg-muted/40' : throttlePressed ? 'w-full' : 'w-0'}`}
          />
        </div>
      </button>
    </div>
  );
};
