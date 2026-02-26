import { useState, useEffect } from "react";
import { Gamepad2, Monitor, Smartphone, Tablet, Laptop } from "lucide-react";

type DeviceType = "android" | "ios" | "ipad" | "mac" | "windows" | "linux" | "chromebook" | "unknown";
export type InputMode = "console" | "device";

interface DeviceSelectionScreenProps {
  onSelect: (mode: InputMode) => void;
}

function detectDevice(): { type: DeviceType; label: string } {
  const ua = navigator.userAgent.toLowerCase();
  
  if (/ipad|macintosh/.test(ua) && "ontouchend" in document) {
    return { type: "ipad", label: "iPad" };
  }
  if (/iphone/.test(ua)) {
    return { type: "ios", label: "iPhone" };
  }
  if (/android/.test(ua) && /mobile/.test(ua)) {
    return { type: "android", label: "Android" };
  }
  if (/android/.test(ua)) {
    return { type: "android", label: "Android Tablet" };
  }
  if (/macintosh|mac os x/.test(ua)) {
    return { type: "mac", label: "Mac" };
  }
  if (/cros/.test(ua)) {
    return { type: "chromebook", label: "Chromebook" };
  }
  if (/windows/.test(ua)) {
    return { type: "windows", label: "Windows PC" };
  }
  if (/linux/.test(ua)) {
    return { type: "linux", label: "Linux PC" };
  }
  return { type: "unknown", label: "This Device" };
}

function getDeviceIcon(type: DeviceType) {
  switch (type) {
    case "android":
    case "ios":
      return Smartphone;
    case "ipad":
      return Tablet;
    case "mac":
    case "windows":
    case "linux":
    case "chromebook":
      return Laptop;
    default:
      return Monitor;
  }
}

export const DeviceSelectionScreen = ({ onSelect }: DeviceSelectionScreenProps) => {
  const [device, setDevice] = useState<{ type: DeviceType; label: string }>({ type: "unknown", label: "This Device" });
  const [hovered, setHovered] = useState<InputMode | null>(null);

  useEffect(() => {
    setDevice(detectDevice());
  }, []);

  const DeviceIcon = getDeviceIcon(device.type);

  return (
    <div className="h-[100dvh] w-full flex flex-col items-center justify-center bg-background relative overflow-hidden">
      {/* Background grid */}
      <div className="absolute inset-0 opacity-5">
        <div className="w-full h-full" style={{
          backgroundImage: 'linear-gradient(hsl(var(--primary) / 0.3) 1px, transparent 1px), linear-gradient(90deg, hsl(var(--primary) / 0.3) 1px, transparent 1px)',
          backgroundSize: '40px 40px'
        }} />
      </div>

      {/* Scanning line */}
      <div className="absolute inset-0 pointer-events-none overflow-hidden">
        <div className="w-full h-px bg-gradient-to-r from-transparent via-primary to-transparent animate-pulse opacity-40"
          style={{ position: 'absolute', top: '25%' }} />
      </div>

      {/* Content */}
      <div className="relative z-10 flex flex-col items-center gap-6 px-6 w-full max-w-lg">
        {/* AMG Logo */}
        <div className="flex items-center gap-2 mb-1">
          <div className="flex gap-1">
            {[...Array(5)].map((_, i) => (
              <div key={i} className="w-1 h-6 bg-gradient-to-b from-muted-foreground to-muted transform -skew-x-12" />
            ))}
          </div>
          <span className="text-foreground font-bold tracking-wider text-xl">AMG</span>
          <span className="text-primary font-bold racing-text text-xl">PETRONAS</span>
        </div>

        <div className="text-muted-foreground racing-text text-xs tracking-[0.3em] uppercase">
          Select Control Mode
        </div>

        {/* Selection cards */}
        <div className="flex gap-3 w-full">
          {/* Console / Game Controller */}
          <button
            onClick={() => onSelect("console")}
            onMouseEnter={() => setHovered("console")}
            onMouseLeave={() => setHovered(null)}
            className={`
              flex-1 flex flex-col items-center justify-center gap-3 p-5 sm:p-6
              border rounded-sm backdrop-blur-md transition-all duration-300
              ${hovered === "console"
                ? "border-primary bg-primary/10 shadow-lg shadow-primary/10"
                : "border-primary/30 bg-card/60 hover:border-primary/60"
              }
            `}
          >
            <Gamepad2 className={`w-10 h-10 sm:w-14 sm:h-14 transition-colors duration-300 ${hovered === "console" ? "text-primary" : "text-muted-foreground"}`} />
            <span className="racing-text text-xs sm:text-sm tracking-widest text-foreground uppercase">Console</span>
            <span className="text-[9px] sm:text-[10px] text-muted-foreground racing-text tracking-wider">VIEW ONLY</span>
          </button>

          {/* This Device */}
          <button
            onClick={() => onSelect("device")}
            onMouseEnter={() => setHovered("device")}
            onMouseLeave={() => setHovered(null)}
            className={`
              flex-1 flex flex-col items-center justify-center gap-3 p-5 sm:p-6
              border rounded-sm backdrop-blur-md transition-all duration-300
              ${hovered === "device"
                ? "border-primary bg-primary/10 shadow-lg shadow-primary/10"
                : "border-primary/30 bg-card/60 hover:border-primary/60"
              }
            `}
          >
            <DeviceIcon className={`w-10 h-10 sm:w-14 sm:h-14 transition-colors duration-300 ${hovered === "device" ? "text-primary" : "text-muted-foreground"}`} />
            <span className="racing-text text-xs sm:text-sm tracking-widest text-foreground uppercase">{device.label}</span>
            <span className="text-[9px] sm:text-[10px] text-muted-foreground racing-text tracking-wider">FULL CONTROL</span>
          </button>
        </div>

        {/* Bottom accent */}
        <div className="flex gap-1 mt-2 opacity-40">
          {[...Array(3)].map((_, i) => (
            <div key={i} className="w-8 h-0.5 bg-primary rounded-full" />
          ))}
        </div>
      </div>
    </div>
  );
};
