import { useState, useEffect } from "react";

export interface AccelerometerHUDProps {
  x: number;
  y: number;
  z: number;
  className?: string;
}

export const AccelerometerHUD = ({ x, y, z, className }: AccelerometerHUDProps) => {
  const formatVal = (v: number) => (v >= 0 ? ` ${v.toFixed(2)}` : v.toFixed(2));
  return (
    <div className={`flex items-center gap-1.5 px-1.5 py-0.5 border border-primary/20 rounded-sm bg-card/40 backdrop-blur-sm ${className}`}>
      <span className="text-[6px] sm:text-[8px] racing-text text-muted-foreground tracking-widest opacity-60">MPU</span>
      <div className="flex gap-1.5 font-mono">
        <span className="text-[8px] sm:text-[10px] text-primary tracking-wider" style={{ filter: 'drop-shadow(0 0 2px hsl(var(--primary)))' }}>
          X{formatVal(x)}
        </span>
        <span className="text-[8px] sm:text-[10px] text-primary tracking-wider" style={{ filter: 'drop-shadow(0 0 2px hsl(var(--primary)))' }}>
          Y{formatVal(y)}
        </span>
        <span className="text-[8px] sm:text-[10px] text-primary tracking-wider" style={{ filter: 'drop-shadow(0 0 2px hsl(var(--primary)))' }}>
          Z{formatVal(z)}
        </span>
      </div>
      <div className="w-1 h-1 rounded-full bg-primary animate-pulse" />
    </div>
  );
}
