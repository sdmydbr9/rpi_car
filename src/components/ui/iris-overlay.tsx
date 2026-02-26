import { useState, useEffect, useRef } from "react";
import { Video } from "lucide-react";

export type IrisPhase = "closed" | "opening" | "open" | "closing";

/**
 * Six-blade iris diaphragm rendered as an SVG-free CSS overlay.
 * `phase` drives CSS transitions on each blade so the browser uses
 * GPU-accelerated compositing for smooth 60 fps animation.
 */
export const IrisOverlay = ({ phase }: { phase: IrisPhase }) => {
  const isOpen = phase === "open";
  const isClosed = phase === "closed";
  const isAnimating = phase === "opening" || phase === "closing";

  // Unmount shortly after fully open to avoid blocking pointer-events.
  const [mounted, setMounted] = useState(true);
  useEffect(() => {
    if (isOpen) {
      const id = setTimeout(() => setMounted(false), 50);
      return () => clearTimeout(id);
    }
    setMounted(true);
  }, [isOpen]);

  if (!mounted) return null;

  const BLADE_COUNT = 6;
  const blades = Array.from({ length: BLADE_COUNT });

  const bladeStyle = (i: number): React.CSSProperties => {
    const angle = (360 / BLADE_COUNT) * i;
    const rad = (angle * Math.PI) / 180;
    const dist = isClosed ? 0 : 160;
    const tx = Math.cos(rad) * dist;
    const ty = Math.sin(rad) * dist;
    return {
      position: "absolute",
      top: "50%",
      left: "50%",
      width: "160%",
      height: "160%",
      background: "black",
      transformOrigin: "center center",
      transform: `translate(-50%, -50%) rotate(${angle}deg) translate(${tx}%, ${ty}%)`,
      transition: isAnimating
        ? "transform 0.7s cubic-bezier(0.4, 0, 0.2, 1)"
        : "none",
      clipPath: "polygon(50% 50%, 30% 0%, 70% 0%)",
    };
  };

  return (
    <div
      className="absolute inset-0 z-30 pointer-events-none overflow-hidden"
      style={{
        opacity: isOpen ? 0 : 1,
        transition: isAnimating ? "opacity 0.3s ease 0.5s" : "none",
      }}
    >
      {/* Solid backdrop that fades as blades slide out */}
      <div
        className="absolute inset-0"
        style={{
          background: "black",
          opacity: isClosed ? 1 : 0,
          transition: isAnimating ? "opacity 0.6s ease" : "none",
        }}
      />

      {/* Iris blades */}
      {blades.map((_, i) => (
        <div key={i} style={bladeStyle(i)} />
      ))}

      {/* Centre label while the iris is still closed */}
      {isClosed && (
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="flex flex-col items-center gap-1">
            <Video className="w-4 h-4 sm:w-6 sm:h-6 text-primary/70" />
            <span className="text-[6px] sm:text-[8px] racing-text text-primary/70 tracking-widest">
              INITIALIZING
            </span>
          </div>
        </div>
      )}
    </div>
  );
};

/**
 * Hook that manages iris phase transitions based on camera enabled state.
 * Returns the current `IrisPhase`.
 */
export function useIrisAnimation(isCameraEnabled: boolean): IrisPhase {
  const [phase, setPhase] = useState<IrisPhase>(isCameraEnabled ? "open" : "closed");
  const prevRef = useRef(isCameraEnabled);

  useEffect(() => {
    const wasEnabled = prevRef.current;
    prevRef.current = isCameraEnabled;

    if (isCameraEnabled && !wasEnabled) {
      // Camera just turned ON → closed → opening → open
      setPhase("closed");
      const t1 = setTimeout(() => setPhase("opening"), 80);
      const t2 = setTimeout(() => setPhase("open"), 900);
      return () => {
        clearTimeout(t1);
        clearTimeout(t2);
      };
    }
    if (!isCameraEnabled && wasEnabled) {
      // Camera just turned OFF → closing → closed
      setPhase("closing");
      const t = setTimeout(() => setPhase("closed"), 750);
      return () => clearTimeout(t);
    }
  }, [isCameraEnabled]);

  return phase;
}
