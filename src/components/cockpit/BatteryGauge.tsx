import { useState, useEffect } from "react";

interface BatteryGaugeProps {
  percentage: number;  // 0-100%
  voltage: number;     // Actual voltage reading
  isEngineRunning?: boolean;
  size?: "small" | "medium" | "large";
}

export const BatteryGauge = ({
  percentage,
  voltage,
  isEngineRunning = false,
  size = "small",
}: BatteryGaugeProps) => {
  const [performSweep, setPerformSweep] = useState(false);
  const [wasEngineRunning, setWasEngineRunning] = useState(false);

  // Trigger needle sweep when engine starts
  useEffect(() => {
    if (isEngineRunning && !wasEngineRunning) {
      setPerformSweep(true);
      setWasEngineRunning(true);
      setTimeout(() => setPerformSweep(false), 800);
    } else if (!isEngineRunning) {
      setWasEngineRunning(false);
    }
  }, [isEngineRunning, wasEngineRunning]);

  // Battery-specific color coding: red < orange < yellow < green
  const getBatteryColor = (pct: number): string => {
    if (pct < 20) return "#ef4444"; // red-500
    if (pct < 40) return "#f97316"; // orange-500
    if (pct < 60) return "#eab308"; // yellow-500
    return "#22c55e"; // green-500
  };

  const getBatteryGlow = (pct: number): string => {
    if (pct < 20) return "drop-shadow(0 0 4px #ef4444)";
    if (pct < 40) return "drop-shadow(0 0 4px #f97316)";
    if (pct < 60) return "drop-shadow(0 0 4px #eab308)";
    return "drop-shadow(0 0 4px #22c55e)";
  };

  const needleRotation = -135 + (percentage / 100) * 270;
  const color = getBatteryColor(percentage);
  const glowEffect = getBatteryGlow(percentage);

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
        stroke="hsl(var(--muted-foreground))"
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

  const voltageSizes = {
    small: "text-[5px] sm:text-[6px]",
    medium: "text-[6px] sm:text-[8px]",
    large: "text-[8px] sm:text-[10px]",
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
          
          {/* Value arc active - Battery colored */}
          <path
            d="M 15 72 A 40 40 0 1 1 85 72"
            fill="none"
            stroke={color}
            strokeWidth="4"
            strokeLinecap="round"
            strokeDasharray="220"
            strokeDashoffset={220 - (220 * percentage) / 100}
            style={{
              filter: percentage > 50 ? glowEffect : 'none',
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
              fill={color}
              style={{
                filter: `drop-shadow(0 0 3px ${color})`
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
          <text x="18" y="68" fill="hsl(var(--muted-foreground))" fontSize="5" textAnchor="middle">0</text>
          <text x="50" y="18" fill="hsl(var(--muted-foreground))" fontSize="5" textAnchor="middle">50</text>
          <text x="82" y="68" fill="hsl(var(--muted-foreground))" fontSize="5" textAnchor="middle">100</text>
        </svg>
        
        {/* Digital readout - Percentage */}
        <div className={`absolute bottom-[18%] left-1/2 -translate-x-1/2 ${textSizes[size]}`}>
          <span className="racing-number font-bold" style={{ color }}>
            {Math.round(percentage)}
          </span>
        </div>
      </div>
      
      {/* Voltage display below gauge */}
      <div className={`${voltageSizes[size]} text-muted-foreground racing-text mt-0.5 text-center`}>
        <span style={{ color }}>{voltage.toFixed(2)}V</span>
      </div>
    </div>
  );
};
