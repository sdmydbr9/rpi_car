import { useState, useRef, useCallback, useMemo, useEffect } from "react";
import { MapPin, RotateCcw, Home, AlertTriangle } from "lucide-react";
import * as socketClient from "@/lib/socketClient";

interface OdometryPoint {
  x: number;
  y: number;
}

interface ObstaclePoint {
  x: number;
  y: number;
  age_s: number;
}

interface OdometryWidgetProps {
  x: number;           // meters
  y: number;           // meters
  headingDeg: number;  // degrees
  vLinear: number;     // m/s
  active: boolean;
  stuckDetected?: boolean;
  slipDetected?: boolean;
  trail?: OdometryPoint[];
  obstacles?: ObstaclePoint[];
}

const TRAIL_MAX = 200;
const MAP_SIZE = 120;   // px
const MAP_SCALE_M = 2;  // meters visible in each direction from center

export const OdometryWidget = ({
  x,
  y,
  headingDeg,
  vLinear,
  active,
  stuckDetected = false,
  slipDetected = false,
  trail = [],
  obstacles = [],
}: OdometryWidgetProps) => {
  const [expanded, setExpanded] = useState(false);
  const trailRef = useRef<OdometryPoint[]>([]);

  // Accumulate trail points (deduplicate close points)
  useEffect(() => {
    if (!active) return;
    const prev = trailRef.current;
    const last = prev[prev.length - 1];
    const dx = last ? x - last.x : Infinity;
    const dy = last ? y - last.y : Infinity;
    if (dx * dx + dy * dy > 0.001) { // > ~3cm movement
      prev.push({ x, y });
      if (prev.length > TRAIL_MAX) prev.shift();
    }
  }, [x, y, active]);

  const distFromStart = useMemo(() => Math.sqrt(x * x + y * y), [x, y]);

  const handleReset = useCallback(() => {
    socketClient.emitOdometryReset();
    trailRef.current = [];
  }, []);

  const handleReturnToStart = useCallback(() => {
    socketClient.emitReturnToStart();
  }, []);

  // Use server-side trail if available, else local accumulation
  const displayTrail = trail.length > 0 ? trail : trailRef.current;

  // Auto-scale: fit all trail points + current position
  const autoScale = useMemo(() => {
    let maxR = MAP_SCALE_M;
    for (const pt of displayTrail) {
      const r = Math.max(Math.abs(pt.x), Math.abs(pt.y));
      if (r > maxR) maxR = r;
    }
    const r = Math.max(Math.abs(x), Math.abs(y));
    if (r > maxR) maxR = r;
    for (const ob of obstacles) {
      const r2 = Math.max(Math.abs(ob.x), Math.abs(ob.y));
      if (r2 > maxR) maxR = r2;
    }
    return maxR * 1.2; // 20% padding
  }, [displayTrail, x, y, obstacles]);

  const toMapX = useCallback((wx: number) => (MAP_SIZE / 2) + (wx / autoScale) * (MAP_SIZE / 2), [autoScale]);
  const toMapY = useCallback((wy: number) => (MAP_SIZE / 2) - (wy / autoScale) * (MAP_SIZE / 2), [autoScale]);

  // SVG trail polyline
  const trailPoints = useMemo(() => {
    return displayTrail.map(pt => `${toMapX(pt.x).toFixed(1)},${toMapY(pt.y).toFixed(1)}`).join(' ');
  }, [displayTrail, toMapX, toMapY]);

  // Car heading arrow (SVG triangle)
  const headingRad = (headingDeg * Math.PI) / 180;
  const carMapX = toMapX(x);
  const carMapY = toMapY(y);
  const arrowLen = 8;
  const tipX = carMapX + arrowLen * Math.sin(headingRad);
  const tipY = carMapY - arrowLen * Math.cos(headingRad);
  const baseL_X = carMapX + 4 * Math.sin(headingRad - 2.4);
  const baseL_Y = carMapY - 4 * Math.cos(headingRad - 2.4);
  const baseR_X = carMapX + 4 * Math.sin(headingRad + 2.4);
  const baseR_Y = carMapY - 4 * Math.cos(headingRad + 2.4);

  if (!active) {
    return (
      <div className="w-full bg-card/50 rounded border border-border/50 p-1 flex items-center justify-center">
        <span className="text-[7px] sm:text-[8px] text-muted-foreground racing-text">ODOMETRY OFF</span>
      </div>
    );
  }

  return (
    <div className="w-full bg-card/50 rounded border border-border p-0.5 flex flex-col gap-0.5">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-0.5">
          <MapPin className="w-2.5 h-2.5 sm:w-3 sm:h-3 text-emerald-400 flex-shrink-0" />
          <span className="racing-text text-[6px] sm:text-[7px] text-muted-foreground whitespace-nowrap">ODOM</span>
        </div>
        <div className="flex items-center gap-0.5">
          {(stuckDetected || slipDetected) && (
            <AlertTriangle className="w-2.5 h-2.5 text-amber-400 animate-pulse" />
          )}
          <button
            onClick={() => setExpanded(e => !e)}
            className="text-[6px] sm:text-[7px] text-muted-foreground hover:text-foreground racing-text"
          >
            {expanded ? '▼' : '▶'}
          </button>
        </div>
      </div>

      {/* Compact readouts (always visible) */}
      <div className="grid grid-cols-2 gap-x-1 gap-y-0">
        <div className="flex justify-between">
          <span className="text-[5px] sm:text-[6px] text-muted-foreground">X</span>
          <span className="racing-text text-[6px] sm:text-[7px] text-emerald-400 font-bold">{x.toFixed(2)}m</span>
        </div>
        <div className="flex justify-between">
          <span className="text-[5px] sm:text-[6px] text-muted-foreground">Y</span>
          <span className="racing-text text-[6px] sm:text-[7px] text-emerald-400 font-bold">{y.toFixed(2)}m</span>
        </div>
        <div className="flex justify-between">
          <span className="text-[5px] sm:text-[6px] text-muted-foreground">HDG</span>
          <span className="racing-text text-[6px] sm:text-[7px] text-cyan-400 font-bold">{headingDeg.toFixed(0)}°</span>
        </div>
        <div className="flex justify-between">
          <span className="text-[5px] sm:text-[6px] text-muted-foreground">DIST</span>
          <span className="racing-text text-[6px] sm:text-[7px] text-amber-400 font-bold">{distFromStart.toFixed(2)}m</span>
        </div>
      </div>

      {/* Status badges */}
      {(stuckDetected || slipDetected) && (
        <div className="flex gap-0.5 justify-center">
          {stuckDetected && (
            <span className="text-[5px] sm:text-[6px] bg-red-500/20 text-red-400 border border-red-500/40 rounded px-0.5 racing-text font-bold animate-pulse">
              STUCK
            </span>
          )}
          {slipDetected && (
            <span className="text-[5px] sm:text-[6px] bg-amber-500/20 text-amber-400 border border-amber-500/40 rounded px-0.5 racing-text font-bold animate-pulse">
              SLIP
            </span>
          )}
        </div>
      )}

      {/* Expanded: Mini-map + controls */}
      {expanded && (
        <>
          {/* Mini-map */}
          <div className="relative mx-auto border border-border/50 rounded overflow-hidden bg-background/80" style={{ width: MAP_SIZE, height: MAP_SIZE }}>
            <svg width={MAP_SIZE} height={MAP_SIZE} className="absolute inset-0">
              {/* Grid lines */}
              <line x1={MAP_SIZE/2} y1={0} x2={MAP_SIZE/2} y2={MAP_SIZE} stroke="hsl(var(--muted-foreground))" strokeWidth="0.5" opacity="0.2" />
              <line x1={0} y1={MAP_SIZE/2} x2={MAP_SIZE} y2={MAP_SIZE/2} stroke="hsl(var(--muted-foreground))" strokeWidth="0.5" opacity="0.2" />

              {/* Start marker */}
              <circle cx={toMapX(0)} cy={toMapY(0)} r={3} fill="hsl(var(--primary))" opacity="0.6" />
              <text x={toMapX(0) + 5} y={toMapY(0) - 3} fill="hsl(var(--primary))" fontSize="6" opacity="0.7">S</text>

              {/* Trail */}
              {trailPoints && (
                <polyline
                  points={trailPoints}
                  fill="none"
                  stroke="hsl(142, 71%, 45%)"
                  strokeWidth="1.5"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  opacity="0.7"
                />
              )}

              {/* Obstacle memory dots */}
              {obstacles.map((ob, i) => (
                <circle
                  key={i}
                  cx={toMapX(ob.x)}
                  cy={toMapY(ob.y)}
                  r={2.5}
                  fill="hsl(0, 72%, 51%)"
                  opacity={Math.max(0.2, 1.0 - ob.age_s / 2.0)}
                />
              ))}

              {/* Car position + heading arrow */}
              <polygon
                points={`${tipX},${tipY} ${baseL_X},${baseL_Y} ${baseR_X},${baseR_Y}`}
                fill="hsl(142, 71%, 45%)"
                stroke="white"
                strokeWidth="0.5"
              />
              <circle cx={carMapX} cy={carMapY} r={2} fill="white" />
            </svg>

            {/* Scale label */}
            <div className="absolute bottom-0.5 right-0.5 text-[5px] text-muted-foreground/60 racing-text">
              ±{autoScale.toFixed(1)}m
            </div>
          </div>

          {/* Velocity */}
          <div className="flex justify-between">
            <span className="text-[5px] sm:text-[6px] text-muted-foreground">V_LIN</span>
            <span className="racing-text text-[6px] sm:text-[7px] text-foreground font-bold">{(vLinear * 100).toFixed(0)} cm/s</span>
          </div>

          {/* Control buttons */}
          <div className="flex gap-0.5">
            <button
              onClick={handleReset}
              className="flex-1 flex items-center justify-center gap-0.5 px-1 py-0.5 rounded border border-border/50 text-[5px] sm:text-[6px] text-muted-foreground hover:text-foreground hover:bg-muted/30 racing-text transition-colors"
            >
              <RotateCcw className="w-2 h-2" />
              RESET
            </button>
            <button
              onClick={handleReturnToStart}
              className="flex-1 flex items-center justify-center gap-0.5 px-1 py-0.5 rounded border border-emerald-500/40 text-[5px] sm:text-[6px] text-emerald-400 hover:bg-emerald-500/10 racing-text transition-colors"
            >
              <Home className="w-2 h-2" />
              RTH
            </button>
          </div>
        </>
      )}
    </div>
  );
};
