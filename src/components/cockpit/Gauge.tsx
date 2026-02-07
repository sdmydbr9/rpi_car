import { useState, useEffect } from "react";

interface GaugeProps {
  value: number;
  min: number;
  max: number;
  label: string;
  unit: string;
  isEngineRunning?: boolean;
  warningThreshold?: number; // Value above which gauge shows warning color
  size?: "small" | "medium" | "large"; // small for first row, medium/large for second row
}

export const Gauge = ({
  value,
  min,
  max,
  label,
  unit,
  isEngineRunning = false,
  warningThreshold,
  size = "medium",
}: GaugeProps) => {
  const [performSweep, setPerformSweep] = useState(false);
  const [wasEngineRunning, setWasEngineRunning] = useState(false);

  // Trigger needle sweep when engine starts
  useEffect(() => {
    if (isEngineRunning && !wasEngineRunning) {
      setPerformSweep(true);
      setWasEngineRunning(true);
      // Stop the animation after it completes
      setTimeout(() => setPerformSweep(false), 800);
    } else if (!isEngineRunning) {
      setWasEngineRunning(false);
    }
  }, [isEngineRunning, wasEngineRunning]);

  const percentage = Math.min(100, Math.max(0, ((value - min) / (max - min)) * 100));
  const needleRotation = -135 + (percentage / 100) * 270;
  
  // Determine if value is in warning range
  const isWarning = warningThreshold !== undefined && value > warningThreshold;

  // Generate tick marks
  const ticks = [];
  for (let i = 0; i <= 10; i++) {
    const tickAngle = -135 + (i / 10) * 270;
    const isMajor = i % 2 === 0;
    ticks.push(
      <line
        key={i}
        x1="50"
        y1={isMajor ? "12" : "15"}
        x2="50"
        y2="20"
        stroke={isWarning && i > 8 ? "hsl(var(--destructive))" : "hsl(var(--muted-foreground))"}
        strokeWidth={isMajor ? "2" : "1"}
        transform={`rotate(${tickAngle} 50 50)`}
      />
    );
  }

  // Determine sizing based on 'size' prop
  const sizeClasses = {
    small: "w-[min(12vw,4rem)]",
    medium: "w-[min(18vw,5rem)]",
    large: "w-[min(24vw,7rem)]",
  };

  const textSizes = {
    small: "text-[6px] sm:text-[8px]",
    medium: "text-[8px] sm:text-xs",
    large: "text-[10px] sm:text-sm",
  };

  const labelSizes = {
    small: "text-[4px] sm:text-[5px]",
    medium: "text-[5px] sm:text-[7px]",
    large: "text-[6px] sm:text-[8px]",
  };

  return (
    <div className="flex flex-col items-center">
      <div className={`relative ${sizeClasses[size]} aspect-square`}>
        <svg className="w-full h-full" viewBox="0 0 100 100">
          {/* Outer bezel */}
          <circle
            cx="50"
            cy="50"
            r="48"
            fill="hsl(var(--card))"
            stroke="hsl(var(--border))"
            strokeWidth="2"
          />
          
          {/* Inner dial face */}
          <circle
            cx="50"
            cy="50"
            r="44"
            fill="hsl(var(--secondary))"
            stroke="hsl(var(--muted))"
            strokeWidth="1"
          />
          
          {/* Value arc background */}
          <path
            d="M 15 72 A 40 40 0 1 1 85 72"
            fill="none"
            stroke="hsl(var(--muted))"
            strokeWidth="4"
            strokeLinecap="round"
          />
          
          {/* Value arc active */}
          <path
            d="M 15 72 A 40 40 0 1 1 85 72"
            fill="none"
            stroke={isWarning ? "hsl(var(--destructive))" : "hsl(var(--primary))"}
            strokeWidth="4"
            strokeLinecap="round"
            strokeDasharray="220"
            strokeDashoffset={220 - (220 * percentage) / 100}
            style={{
              filter: percentage > 50 ? `drop-shadow(0 0 4px ${isWarning ? 'hsl(var(--destructive))' : 'hsl(var(--primary))'})` : 'none',
              transition: 'stroke-dashoffset 0.15s ease-out'
            }}
          />
          
          {/* Tick marks */}
          {ticks}
          
          {/* Needle */}
          <g transform={`rotate(${performSweep ? -135 : needleRotation} 50 50)`} style={{ 
            transition: performSweep ? 'none' : 'transform 0.1s ease-out',
            animation: performSweep ? 'needleSweep 0.8s ease-in-out' : 'none'
          }}>
            <polygon
              points="50,15 48,50 52,50"
              fill={isWarning ? "hsl(var(--destructive))" : "hsl(var(--primary))"}
              style={{
                filter: `drop-shadow(0 0 3px ${isWarning ? 'hsl(var(--destructive))' : 'hsl(var(--primary))'})`
              }}
            />
          </g>
          
          {/* Center cap */}
          <circle
            cx="50"
            cy="50"
            r="6"
            fill="hsl(var(--muted))"
            stroke="hsl(var(--border))"
            strokeWidth="1"
          />
          
          {/* Min, Mid, Max numbers */}
          <text x="18" y="68" fill="hsl(var(--muted-foreground))" fontSize="5" textAnchor="middle">{min}</text>
          <text x="50" y="18" fill="hsl(var(--muted-foreground))" fontSize="5" textAnchor="middle">{Math.round((min + max) / 2)}</text>
          <text x="82" y="68" fill={isWarning ? "hsl(var(--destructive))" : "hsl(var(--muted-foreground))"} fontSize="5" textAnchor="middle">{max}</text>
        </svg>
        
        {/* Digital readout */}
        <div className={`absolute bottom-[18%] left-1/2 -translate-x-1/2 ${textSizes[size]}`}>
          <span className={`racing-number font-bold ${isWarning ? 'text-destructive text-glow-red' : 'text-foreground'}`}>
            {Math.round(value * 10) / 10}
          </span>
        </div>
      </div>
      
      {/* Label */}
      <div className={`${labelSizes[size]} text-muted-foreground racing-text text-center`}>
        <div>{label}</div>
        <div className="text-[3px] sm:text-[4px]">{unit}</div>
      </div>
    </div>
  );
};
