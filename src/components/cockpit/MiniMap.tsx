import { useState, useCallback, useMemo, useRef, useEffect } from "react";
import { Crosshair, RotateCcw, Locate, Navigation, Home } from "lucide-react";
import { useOdometry, type OdometryPoint } from "@/hooks/useOdometry";
import { emitReturnToStart } from "@/lib/socketClient";

interface MiniMapProps {
  className?: string;
}

const MAP_SIZE = 300; // logical SVG size
const DEFAULT_SCALE = 75; // pixels per meter (shows ~4m range)
const MIN_SCALE = 15;
const MAX_SCALE = 300;

export const MiniMap = ({ className = "" }: MiniMapProps) => {
  const { odometry, trail, distanceFromStart, resetTrail, rthActive } = useOdometry();
  const [autoFollow, setAutoFollow] = useState(true);
  const [scale, setScale] = useState(DEFAULT_SCALE);
  const [panOffset, setPanOffset] = useState({ x: 0, y: 0 });
  const svgRef = useRef<SVGSVGElement>(null);
  const isPanning = useRef(false);
  const panStart = useRef({ x: 0, y: 0 });

  // Auto-scale is intentionally NOT run on every trail update.
  // The user controls zoom via scroll wheel; autoFollow only re-centres.

  const center = useMemo(() => {
    if (autoFollow) {
      return { x: odometry.x, y: odometry.y };
    }
    return { x: panOffset.x, y: panOffset.y };
  }, [autoFollow, odometry.x, odometry.y, panOffset]);

  const worldToScreen = useCallback(
    (wx: number, wy: number) => ({
      x: MAP_SIZE / 2 + (wx - center.x) * scale,
      y: MAP_SIZE / 2 - (wy - center.y) * scale, // Y-up in world, Y-down in SVG
    }),
    [center, scale]
  );

  const handleRecenter = () => {
    setAutoFollow(true);
    setPanOffset({ x: 0, y: 0 });
    setScale(DEFAULT_SCALE);
  };

  const handlePointerDown = (e: React.PointerEvent) => {
    if (autoFollow) {
      // Sync panOffset to current rover position so the view doesn't jump
      setPanOffset({ x: odometry.x, y: odometry.y });
      setAutoFollow(false);
    }
    isPanning.current = true;
    panStart.current = { x: e.clientX, y: e.clientY };
    (e.target as Element).setPointerCapture?.(e.pointerId);
  };

  const handlePointerMove = (e: React.PointerEvent) => {
    if (!isPanning.current) return;
    const dx = (e.clientX - panStart.current.x) / scale;
    const dy = (e.clientY - panStart.current.y) / scale;
    setPanOffset((prev) => ({ x: prev.x - dx, y: prev.y + dy }));
    panStart.current = { x: e.clientX, y: e.clientY };
  };

  const handlePointerUp = () => {
    isPanning.current = false;
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    if (autoFollow) {
      // Sync panOffset to current rover position so the view doesn't jump
      setPanOffset({ x: odometry.x, y: odometry.y });
      setAutoFollow(false);
    }
    setScale((s) => Math.min(MAX_SCALE, Math.max(MIN_SCALE, s * (e.deltaY > 0 ? 0.9 : 1.1))));
  };

  // Full trail path + live segment to current position
  const trailPath = useMemo(() => {
    if (trail.length < 1) return "";
    return trail
      .map((p, i) => {
        const s = worldToScreen(p.x, p.y);
        return `${i === 0 ? "M" : "L"}${s.x.toFixed(1)},${s.y.toFixed(1)}`;
      })
      .join(" ");
  }, [trail, worldToScreen]);

  // Live segment from last trail point to current rover position
  const liveSegment = useMemo(() => {
    if (trail.length < 1) return "";
    const last = trail[trail.length - 1];
    const s1 = worldToScreen(last.x, last.y);
    const s2 = worldToScreen(odometry.x, odometry.y);
    return `M${s1.x.toFixed(1)},${s1.y.toFixed(1)} L${s2.x.toFixed(1)},${s2.y.toFixed(1)}`;
  }, [trail, odometry.x, odometry.y, worldToScreen]);

  // Compass ticks and labels around the boundary
  const COMPASS_RADIUS = MAP_SIZE / 2 - 1;
  const COMPASS_CX = MAP_SIZE / 2;
  const COMPASS_CY = MAP_SIZE / 2;
  const CARDINAL: { label: string; deg: number }[] = [
    { label: "N", deg: 0 }, { label: "E", deg: 90 },
    { label: "S", deg: 180 }, { label: "W", deg: 270 },
  ];
  const TICK_DEGS = Array.from({ length: 36 }, (_, i) => i * 10); // every 10°

  const originScreen = worldToScreen(0, 0);
  const roverScreen = worldToScreen(odometry.x, odometry.y);

  // Grid lines
  const gridLines = useMemo(() => {
    const lines: { x1: number; y1: number; x2: number; y2: number; isAxis?: boolean }[] = [];
    const gridSpacing = scale > 100 ? 0.25 : scale > 40 ? 0.5 : 1.0;
    const viewRange = MAP_SIZE / scale / 2;

    for (
      let w = Math.floor((center.x - viewRange) / gridSpacing) * gridSpacing;
      w <= center.x + viewRange;
      w += gridSpacing
    ) {
      const sx = MAP_SIZE / 2 + (w - center.x) * scale;
      const isAxis = Math.abs(w) < 0.001;
      lines.push({ x1: sx, y1: 0, x2: sx, y2: MAP_SIZE, isAxis });
    }
    for (
      let w = Math.floor((center.y - viewRange) / gridSpacing) * gridSpacing;
      w <= center.y + viewRange;
      w += gridSpacing
    ) {
      const sy = MAP_SIZE / 2 - (w - center.y) * scale;
      const isAxis = Math.abs(w) < 0.001;
      lines.push({ x1: 0, y1: sy, x2: MAP_SIZE, y2: sy, isAxis });
    }
    return lines;
  }, [center, scale]);

  const formatNum = (n: number) => n.toFixed(2);
  const gridLabel = scale > 100 ? "0.25m" : scale > 40 ? "0.5m" : "1.0m";

  return (
    <div className={`flex flex-col h-full w-full ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between px-2 py-1 border-b border-border/30">
        <div className="flex items-center gap-1.5">
          <Navigation className="w-3 h-3 text-primary" />
          <span className="text-[8px] sm:text-[10px] racing-text text-primary">ODOMETRY MAP</span>
          {!odometry.active && (
            <span className="text-[7px] px-1 py-0.5 rounded bg-destructive/20 text-destructive border border-destructive/30 racing-text">
              INACTIVE
            </span>
          )}
        </div>
        <div className="flex items-center gap-1">
          <button
            onClick={() => setAutoFollow(!autoFollow)}
            className={`p-0.5 rounded transition-colors ${
              autoFollow
                ? "bg-primary/20 text-primary"
                : "text-muted-foreground hover:text-foreground"
            }`}
            title={autoFollow ? "Auto-follow ON" : "Auto-follow OFF"}
          >
            <Locate className="w-3 h-3" />
          </button>
          <button
            onClick={handleRecenter}
            className="p-0.5 rounded text-muted-foreground hover:text-foreground transition-colors"
            title="Recenter"
          >
            <Crosshair className="w-3 h-3" />
          </button>
          <button
            onClick={resetTrail}
            className="p-0.5 rounded text-muted-foreground hover:text-destructive transition-colors"
            title="Reset trail"
          >
            <RotateCcw className="w-3 h-3" />
          </button>
        </div>
      </div>

      {/* Map Canvas */}
      <div className="flex-1 relative min-h-0 overflow-hidden">
        {!odometry.active ? (
          <div className="absolute inset-0 flex items-center justify-center">
            <div className="text-center">
              <Navigation className="w-6 h-6 text-muted-foreground/30 mx-auto mb-1" />
              <p className="text-[8px] sm:text-[10px] text-muted-foreground racing-text">
                ODOMETRY INACTIVE
              </p>
              <p className="text-[7px] text-muted-foreground/50">
                Waiting for sensor data…
              </p>
            </div>
          </div>
        ) : (
          <svg
            ref={svgRef}
            viewBox={`0 0 ${MAP_SIZE} ${MAP_SIZE}`}
            className="w-full h-full touch-none select-none"
            onPointerDown={handlePointerDown}
            onPointerMove={handlePointerMove}
            onPointerUp={handlePointerUp}
            onPointerLeave={handlePointerUp}
            onWheel={handleWheel}
          >
            <defs>
              <clipPath id="mapClip">
                <circle cx={COMPASS_CX} cy={COMPASS_CY} r={COMPASS_RADIUS - 12} />
              </clipPath>
            </defs>

            {/* Compass ring background (symmetric, no rotation needed) */}
            <circle cx={COMPASS_CX} cy={COMPASS_CY} r={COMPASS_RADIUS} fill="none"
              stroke="hsl(var(--border) / 0.4)" strokeWidth={24} />

            {/* Rotating compass rose — rotates so current heading is at top */}
            {/* heading is math convention (0=East, CCW+). To put car heading at 12 o'clock:
                compassBearing = 90 - heading, rotation = -compassBearing = heading - 90 */}
            <g
              transform={`rotate(${odometry.heading - 90}, ${COMPASS_CX}, ${COMPASS_CY})`}
            >
              {/* Degree ticks around perimeter */}
              {TICK_DEGS.map((deg) => {
                const isCardinal = deg % 90 === 0;
                const is30 = deg % 30 === 0;
                // 0° = North = top of SVG
                const rad = ((deg - 90) * Math.PI) / 180;
                const outerR = COMPASS_RADIUS;
                const innerR = isCardinal ? COMPASS_RADIUS - 10 : is30 ? COMPASS_RADIUS - 7 : COMPASS_RADIUS - 4;
                const x1 = COMPASS_CX + Math.cos(rad) * innerR;
                const y1 = COMPASS_CY + Math.sin(rad) * innerR;
                const x2 = COMPASS_CX + Math.cos(rad) * outerR;
                const y2 = COMPASS_CY + Math.sin(rad) * outerR;
                return (
                  <line key={`t${deg}`} x1={x1} y1={y1} x2={x2} y2={y2}
                    stroke={isCardinal ? "hsl(var(--primary) / 0.8)" : is30 ? "hsl(var(--muted-foreground) / 0.5)" : "hsl(var(--muted-foreground) / 0.25)"}
                    strokeWidth={isCardinal ? 1.5 : 0.5} />
                );
              })}

              {/* Cardinal labels N / E / S / W — counter-rotated to stay upright */}
              {CARDINAL.map(({ label, deg }) => {
                const rad = ((deg - 90) * Math.PI) / 180;
                const labelR = COMPASS_RADIUS - 17;
                const tx = COMPASS_CX + Math.cos(rad) * labelR;
                const ty = COMPASS_CY + Math.sin(rad) * labelR;
                return (
                  <text key={label} x={tx} y={ty}
                    textAnchor="middle" dominantBaseline="central"
                    transform={`rotate(${90 - odometry.heading}, ${tx}, ${ty})`}
                    fill={label === "N" ? "hsl(var(--destructive))" : "hsl(var(--primary))"}
                    fontSize="9" fontFamily="Rajdhani, sans-serif" fontWeight="700"
                    opacity={0.9}>
                    {label}
                  </text>
                );
              })}

              {/* Degree numbers at 30° intervals (skip cardinals) — counter-rotated */}
              {TICK_DEGS.filter((d) => d % 30 === 0 && d % 90 !== 0).map((deg) => {
                const rad = ((deg - 90) * Math.PI) / 180;
                const labelR = COMPASS_RADIUS - 17;
                const tx = COMPASS_CX + Math.cos(rad) * labelR;
                const ty = COMPASS_CY + Math.sin(rad) * labelR;
                return (
                  <text key={`d${deg}`} x={tx} y={ty}
                    textAnchor="middle" dominantBaseline="central"
                    transform={`rotate(${90 - odometry.heading}, ${tx}, ${ty})`}
                    fill="hsl(var(--muted-foreground))"
                    fontSize="6" fontFamily="Rajdhani, sans-serif" fontWeight="600"
                    opacity={0.7}>
                    {deg}
                  </text>
                );
              })}
            </g>

            {/* Fixed heading indicator triangle at top (always at 12 o'clock) */}
            <polygon
              points={`${COMPASS_CX},${COMPASS_CY - COMPASS_RADIUS + 12} ${COMPASS_CX - 4},${COMPASS_CY - COMPASS_RADIUS + 2} ${COMPASS_CX + 4},${COMPASS_CY - COMPASS_RADIUS + 2}`}
              fill="hsl(var(--primary))" stroke="hsl(var(--primary-foreground) / 0.5)" strokeWidth={0.5}
            />

            {/* Map content clipped to inner circle */}
            <g clipPath="url(#mapClip)">
              {/* Grid */}
              {gridLines.map((l, i) => (
                <line
                  key={i}
                  x1={l.x1} y1={l.y1} x2={l.x2} y2={l.y2}
                  stroke={l.isAxis ? "hsl(var(--primary) / 0.25)" : "hsl(var(--border) / 0.3)"}
                  strokeWidth={l.isAxis ? 1 : 0.5}
                />
              ))}

              {/* Trail path */}
              {trailPath && (
                <path d={trailPath} fill="none"
                  stroke="hsl(var(--primary))" strokeWidth={2}
                  strokeLinecap="round" strokeLinejoin="round" opacity={0.8} />
              )}
              {/* Trail glow for visibility */}
              {trailPath && (
                <path d={trailPath} fill="none"
                  stroke="hsl(var(--primary))" strokeWidth={4}
                  strokeLinecap="round" strokeLinejoin="round" opacity={0.15} />
              )}

              {/* Live segment to current position */}
              {liveSegment && (
                <path d={liveSegment} fill="none"
                  stroke="hsl(var(--primary))" strokeWidth={2}
                  strokeLinecap="round" opacity={1}
                  strokeDasharray="4 2" />
              )}

              {/* Origin marker */}
              <circle cx={originScreen.x} cy={originScreen.y} r={4}
                fill="none" stroke="hsl(var(--accent))" strokeWidth={1.5} opacity={0.8} />
              <circle cx={originScreen.x} cy={originScreen.y} r={1.5}
                fill="hsl(var(--accent))" opacity={0.9} />
              <text x={originScreen.x + 6} y={originScreen.y - 5}
                fill="hsl(var(--accent))" fontSize="7"
                fontFamily="Rajdhani, sans-serif" fontWeight="600" opacity={0.8}>
                START
              </text>

              {/* RTH return line: rover → origin */}
              {rthActive && (
                <line
                  x1={roverScreen.x} y1={roverScreen.y}
                  x2={originScreen.x} y2={originScreen.y}
                  stroke="hsl(var(--destructive))"
                  strokeWidth={1.5} strokeDasharray="6 3"
                  opacity={0.7}
                />
              )}

              {/* Rover position pulse */}
              <circle cx={roverScreen.x} cy={roverScreen.y} r={8}
                fill="none" stroke="hsl(var(--primary))" strokeWidth={1} opacity={0.4}>
                <animate attributeName="r" values="6;12;6" dur="2s" repeatCount="indefinite" />
                <animate attributeName="opacity" values="0.4;0.1;0.4" dur="2s" repeatCount="indefinite" />
              </circle>

              {/* Rover arrow */}
              <g transform={`translate(${roverScreen.x}, ${roverScreen.y}) rotate(${-odometry.heading + 90})`}>
                <path d="M0,-12 L-6,4 L6,4 Z"
                  fill="hsl(var(--primary) / 0.15)" stroke="none" />
                <path d="M0,-8 L-4,4 L0,2 L4,4 Z"
                  fill="hsl(var(--primary))"
                  stroke="hsl(var(--primary-foreground) / 0.3)" strokeWidth={0.5} />
                <circle r={1.5} fill="hsl(var(--primary-foreground))" />
              </g>
            </g>

            {/* Grid scale label */}
            <text
              x={MAP_SIZE - 4}
              y={MAP_SIZE - 4}
              textAnchor="end"
              fill="hsl(var(--muted-foreground))"
              fontSize="7"
              fontFamily="Rajdhani, sans-serif"
              opacity={0.6}
            >
              grid: {gridLabel}
            </text>
          </svg>
        )}
      </div>

      {/* Telemetry Footer */}
      <div className="flex items-center gap-0.5 px-1.5 py-1 border-t border-border/30 bg-card/50">
        <div className="grid grid-cols-4 gap-0.5 flex-1">
          <TelemetryCell label="X" value={`${formatNum(odometry.x)}m`} />
          <TelemetryCell label="Y" value={`${formatNum(odometry.y)}m`} />
          <TelemetryCell label="HDG" value={`${(((90 - odometry.heading) % 360 + 360) % 360).toFixed(0)}°`} />
          <TelemetryCell label="DIST" value={`${formatNum(distanceFromStart)}m`} active />
        </div>
        <button
          onClick={emitReturnToStart}
          className={`flex items-center gap-1 px-2 py-1 rounded border transition-colors flex-shrink-0 ${
            rthActive
              ? "bg-destructive/20 text-destructive border-destructive/30 animate-pulse"
              : "bg-primary/10 hover:bg-primary/25 text-primary border-primary/30"
          }`}
          title={rthActive ? "Cancel Return to Home" : "Return to Home"}
        >
          <Home className="w-3 h-3" />
          <span className="text-[8px] sm:text-[10px] racing-text font-semibold">
            {rthActive ? "STOP" : "RTH"}
          </span>
        </button>
      </div>
    </div>
  );
};

const TelemetryCell = ({
  label,
  value,
  active = false,
}: {
  label: string;
  value: string;
  active?: boolean;
}) => (
  <div className="text-center">
    <div className="text-[6px] sm:text-[7px] text-muted-foreground racing-text leading-none">
      {label}
    </div>
    <div
      className={`text-[8px] sm:text-[10px] font-bold racing-number leading-tight ${
        active ? "text-primary" : "text-foreground"
      }`}
    >
      {value}
    </div>
  </div>
);
