/**
 * CompassRose — North-up compass.
 *
 * The rose (ticks, N/S/E/W labels) is FIXED — North is always at the top.
 * A cyan heading arrow rotates by +heading to show the car's current bearing.
 * A dimmer cyan triangle marks the target heading the PID is holding.
 *
 * CW-positive bearing (0 = N, 90 = E, 180 = S, 270 = W).
 */

export interface CompassRoseProps {
  heading: number;
  targetHeading: number;
  pidCorrection?: number;
  className?: string;
}

function cardinalLabel(deg: number): string {
  const labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
  return labels[Math.round(((deg % 360) + 360) % 360 / 45) % 8];
}

export const CompassRose = ({
  heading,
  targetHeading,
  pidCorrection = 0,
  className = "",
}: CompassRoseProps) => {
  const displayHeading = ((heading % 360) + 360) % 360;
  const displayTarget  = ((targetHeading % 360) + 360) % 360;

  // Lubber-line / crosshair colour: cyan on-track → amber → red
  const absCorr = Math.abs(pidCorrection);
  const corrColor =
    absCorr < 3  ? "hsl(var(--primary))" :
    absCorr < 10 ? "#f59e0b" :
                   "#ef4444";

  return (
    <div className={`flex flex-col items-center ${className}`}>
      {/* ── SVG rose ─────────────────────────────────────────────── */}
      <div className="relative w-[min(20vw,6rem)] aspect-square">
        <svg viewBox="0 0 100 100" className="w-full h-full">

          {/* Outer bezel rings */}
          <circle cx="50" cy="50" r="47" fill="none"
            stroke="hsl(var(--primary) / 0.3)" strokeWidth="1.5" />
          <circle cx="50" cy="50" r="44" fill="hsl(var(--card) / 0.6)"
            stroke="hsl(var(--border) / 0.5)" strokeWidth="0.5" />

          {/* ── Fixed rose (North always at top) ─────────────────── */}
          <g>

            {/* 5° minor ticks */}
            {Array.from({ length: 72 }, (_, i) => i * 5).map(deg => (
              <line key={deg} x1="50" y1="6" x2="50" y2="9"
                stroke="hsl(var(--muted-foreground) / 0.3)" strokeWidth="0.5"
                transform={`rotate(${deg} 50 50)`} />
            ))}

            {/* 10° medium ticks */}
            {Array.from({ length: 36 }, (_, i) => i * 10)
              .filter(d => d % 45 !== 0)
              .map(deg => (
                <line key={deg} x1="50" y1="5" x2="50" y2="9"
                  stroke="hsl(var(--muted-foreground) / 0.5)" strokeWidth="0.8"
                  transform={`rotate(${deg} 50 50)`} />
              ))}

            {/* 45° major ticks */}
            {[0, 45, 90, 135, 180, 225, 270, 315].map(deg => (
              <line key={deg} x1="50" y1="4" x2="50" y2="12"
                stroke="hsl(var(--primary) / 0.7)" strokeWidth="1.4"
                transform={`rotate(${deg} 50 50)`} />
            ))}

            {/* Intercardinal labels */}
            {([{l:"NE",d:45},{l:"SE",d:135},{l:"SW",d:225},{l:"NW",d:315}] as const).map(({l,d}) => (
              <text key={l} x="50" y="20"
                textAnchor="middle" dominantBaseline="central"
                fontSize="5" fontFamily="monospace"
                fill="hsl(var(--muted-foreground) / 0.55)"
                className="select-none"
                transform={`rotate(${d} 50 50)`}>
                {l}
              </text>
            ))}

            {/* Cardinal labels — N=red, E/S/W=primary */}
            {([{l:"N",d:0},{l:"E",d:90},{l:"S",d:180},{l:"W",d:270}] as const).map(({l,d}) => (
              <text key={l} x="50" y="23"
                textAnchor="middle" dominantBaseline="central"
                fontSize="8" fontWeight="bold"
                fill={l === "N" ? "hsl(var(--destructive))" : "hsl(var(--primary))"}
                className="select-none"
                transform={`rotate(${d} 50 50)`}
                style={l === "N" ? { filter: "drop-shadow(0 0 3px hsl(var(--destructive)))" } : undefined}>
                {l}
              </text>
            ))}

            {/* North pointer triangle (red, fixed at top) */}
            <polygon points="50,6 47,13 53,13"
              fill="hsl(var(--destructive))"
              style={{ filter: "drop-shadow(0 0 4px hsl(var(--destructive)))" }} />

          </g>

          {/* ── Target heading indicator (dim cyan triangle, rotates to target) ── */}
          <g style={{
            transform: `rotate(${displayTarget}deg)`,
            transformOrigin: "50px 50px",
            transition: "transform 0.3s ease-out",
          }}>
            <polygon points="50,6 47.5,13 52.5,13"
              fill="hsl(var(--primary) / 0.35)"
              stroke="hsl(var(--primary) / 0.6)" strokeWidth="0.6"
              style={{ filter: "drop-shadow(0 0 2px hsl(var(--primary)))" }} />
          </g>

          {/* ── Heading arrow (rotates to current bearing) ────────── */}
          <g style={{
            transform: `rotate(${displayHeading}deg)`,
            transformOrigin: "50px 50px",
            transition: "transform 0.15s ease-out",
          }}>
            {/* Arrow shaft */}
            <line x1="50" y1="16" x2="50" y2="34"
              stroke={corrColor} strokeWidth="2"
              style={{ filter: `drop-shadow(0 0 3px ${corrColor})` }} />
            {/* Arrowhead pointing outward (toward bearing on ring) */}
            <polygon points="50,4 46.5,14 53.5,14"
              fill={corrColor}
              style={{ filter: `drop-shadow(0 0 4px ${corrColor})` }} />
            {/* Tail stub (opposite direction) */}
            <line x1="50" y1="34" x2="50" y2="40"
              stroke={corrColor} strokeWidth="1" strokeDasharray="1.5,1.5"
              opacity="0.5" />
          </g>

          {/* Digital heading — centre of dial */}
          <text x="50" y="57" textAnchor="middle" dominantBaseline="central"
            fontSize="10" fontFamily="monospace"
            fill="hsl(var(--primary))"
            style={{ filter: "drop-shadow(0 0 3px hsl(var(--primary)))" }}
            className="select-none">
            {Math.round(displayHeading).toString().padStart(3, "0")}°
          </text>

          {/* Centre hub dot */}
          <circle cx="50" cy="50" r="2.5"
            fill="hsl(var(--card))"
            stroke={corrColor} strokeWidth="1.2"
            style={{ filter: `drop-shadow(0 0 2px ${corrColor})` }} />

        </svg>
      </div>

      {/* ── Text readout below dial ─────────────────────────────── */}
      <div className="flex items-center gap-1.5 mt-0.5 font-mono">
        <span className="text-[7px] sm:text-[9px] text-primary tracking-wider"
          style={{ filter: "drop-shadow(0 0 2px hsl(var(--primary)))" }}>
          {cardinalLabel(displayHeading)}
        </span>
        <span className="text-[5px] sm:text-[7px] racing-text text-muted-foreground opacity-60">HDG</span>
        <span className="text-[5px] sm:text-[6px] text-primary/55">
          TGT {Math.round(displayTarget).toString().padStart(3, "0")}°
        </span>
        {absCorr > 1 && (
          <span className="text-[6px] sm:text-[7px]" style={{ color: corrColor }}>
            {pidCorrection > 0 ? "►" : "◄"}{absCorr.toFixed(1)}
          </span>
        )}
        <div className="w-1 h-1 rounded-full bg-primary animate-pulse" />
      </div>
    </div>
  );
};
