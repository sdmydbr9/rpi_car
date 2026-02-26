import { useState, useCallback, useMemo, useRef, useEffect } from "react";
import { X, Wifi, Zap, Power, VideoOff, Camera, Mic, Gamepad2 } from "lucide-react";
import { useGameFeedback } from "@/hooks/useGameFeedback";
import { useTouchTracking } from "@/hooks/useTouchTracking";
import { IrisOverlay, useIrisAnimation } from "@/components/ui/iris-overlay";
import type { SpeedUnit } from "@/components/cockpit/Speedometer";

const SPEED_UNIT_LABELS: Record<SpeedUnit, string> = {
  "m/min": "M/MIN",
  "cm/s": "CM/S",
  "km/h": "KM/H",
  "mph": "MPH",
};

const convertMpm = (mpm: number, unit: SpeedUnit): number => {
  switch (unit) {
    case "cm/s":  return mpm * 100 / 60;
    case "km/h":  return mpm * 60 / 1000;
    case "mph":   return mpm * 60 / 1609.344;
    default:      return mpm; // m/min
  }
};

interface ImmersiveHUDProps {
  isOpen: boolean;
  onClose: () => void;
  streamUrl?: string;
  speed: number;
  speedMpm?: number;
  speedUnit?: SpeedUnit;
  gear: string;
  throttle: boolean;
  brake: boolean;
  isConnected: boolean;
  isAutoMode: boolean;
  isEmergencyStop: boolean;
  eBrakeActive: boolean;
  onThrottleChange: (active: boolean) => void;
  onBrakeChange: (active: boolean) => void;
  onEmergencyStop: () => void;
  onEBrakeToggle: () => void;
  onAutoModeToggle: () => void;
  onSteeringChange?: (angle: number) => void;
  onGearChange?: (gear: string) => void;
  steeringAngle?: number;
  // Camera config for HUD badge
  cameraResolution?: string;
  cameraJpegQuality?: number;
  cameraFramerate?: number;
  cameraActualFps?: number;
  visionActive?: boolean;
  visionFps?: number;
  isCameraEnabled?: boolean;
  userWantsVision?: boolean;
  onToggleCamera?: () => void;
  // AI Narration overlay
  narrationEnabled?: boolean;
  narrationSpeaking?: boolean;
  narrationLastText?: string;
  // View-only mode (console/gamepad)
  viewOnly?: boolean;
  gamepadConnected?: boolean;
  isEngineRunning?: boolean;
  inputMode?: "console" | "device" | null;
}

export const ImmersiveHUD = ({
  isOpen,
  onClose,
  streamUrl,
  speed,
  speedMpm = 0,
  speedUnit = "m/min",
  gear,
  throttle,
  brake,
  isConnected,
  isAutoMode,
  isEmergencyStop,
  eBrakeActive,
  onThrottleChange,
  onBrakeChange,
  onEmergencyStop,
  onEBrakeToggle,
  onAutoModeToggle,
  onSteeringChange,
  onGearChange,
  steeringAngle = 0,
  cameraResolution,
  cameraJpegQuality,
  cameraFramerate,
  cameraActualFps,
  visionActive,
  visionFps,
  isCameraEnabled,
  userWantsVision,
  onToggleCamera,
  narrationEnabled,
  narrationSpeaking,
  narrationLastText,
  viewOnly = false,
  gamepadConnected = false,
  isEngineRunning = false,
  inputMode = null,
}: ImmersiveHUDProps) => {
  const { triggerHaptic, playSound } = useGameFeedback();
  const [steeringDirection, setSteeringDirection] = useState<'left' | 'right' | null>(null);
  const [feedViewerKey, setFeedViewerKey] = useState(0);

  /* ---- Iris lens animation ---- */
  const irisPhase = useIrisAnimation(isCameraEnabled ?? false);
  
  // Progressive steering constants
  const STEER_INCREMENT = 5;       // degrees per tick
  const STEER_INTERVAL_MS = 100;   // ms between ticks (50°/s ramp)
  const steerIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const internalAngleRef = useRef<number>(0);

  // Sync internal angle ref with prop so release always resets properly
  useEffect(() => {
    internalAngleRef.current = steeringAngle;
  }, [steeringAngle]);

  // Cleanup interval on unmount
  useEffect(() => {
    return () => {
      if (steerIntervalRef.current) clearInterval(steerIntervalRef.current);
    };
  }, []);

  useEffect(() => {
    setFeedViewerKey((k) => k + 1);
  }, [streamUrl, isCameraEnabled]);

  // Refs for multitouch-tracked interactive zones
  const steerLeftRef = useRef<HTMLButtonElement>(null);
  const steerRightRef = useRef<HTMLButtonElement>(null);
  const hudBrakeRef = useRef<HTMLButtonElement>(null);
  const hudThrottleRef = useRef<HTMLButtonElement>(null);
  
  // Resolution label map
  const resolutionLabel = useMemo(() => {
    const map: Record<string, string> = { low: '640×480', medium: '1280×720', high: '1920×1080' };
    return map[cameraResolution || 'low'] || cameraResolution || '640×480';
  }, [cameraResolution]);
  
  // RPM simulation based on speed and throttle
  const rpm = Math.min(100, (speed / 100) * 80 + (throttle ? 20 : 0));
  const effectiveViewerUrl = useMemo(() => {
    if (!streamUrl) return undefined;
    return `${streamUrl}${streamUrl.includes('?') ? '&' : '?'}autoplay=1&muted=1&controls=0&playsinline=1&disablepictureinpicture=1`;
  }, [streamUrl]);
  const isRedline = rpm > 85;

  // Converted speed for HUD display
  const hudDisplaySpeed = useMemo(() => convertMpm(speedMpm, speedUnit), [speedMpm, speedUnit]);
  const hudSpeedLabel = SPEED_UNIT_LABELS[speedUnit] || "M/MIN";

  // Narration auto-fade: show text while speaking, fade out after speech ends
  const [narrationVisible, setNarrationVisible] = useState(false);
  const [narrationFading, setNarrationFading] = useState(false);
  const fadeTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    if (narrationSpeaking && narrationLastText) {
      // New speech started — show immediately, cancel any pending fade
      if (fadeTimerRef.current) clearTimeout(fadeTimerRef.current);
      setNarrationVisible(true);
      setNarrationFading(false);
    } else if (!narrationSpeaking && narrationVisible) {
      // Speech ended — start fade out after a short delay
      setNarrationFading(true);
      fadeTimerRef.current = setTimeout(() => {
        setNarrationVisible(false);
        setNarrationFading(false);
      }, 3000); // 3s fade-out duration
    }
    return () => {
      if (fadeTimerRef.current) clearTimeout(fadeTimerRef.current);
    };
  }, [narrationSpeaking, narrationLastText]);

  const handleBrakeStart = useCallback(() => {
    triggerHaptic('heavy');
    onBrakeChange(true);
  }, [triggerHaptic, onBrakeChange]);
  
  const handleBrakeEnd = useCallback(() => {
    onBrakeChange(false);
  }, [onBrakeChange]);
  
  const handleThrottleStart = useCallback(() => {
    triggerHaptic('medium');
    onThrottleChange(true);
  }, [triggerHaptic, onThrottleChange]);
  
  const handleThrottleEnd = useCallback(() => {
    onThrottleChange(false);
  }, [onThrottleChange]);
  
  const handleEBrake = useCallback(() => {
    triggerHaptic('heavy');
    playSound('emergency');
    onEBrakeToggle();
  }, [triggerHaptic, playSound, onEBrakeToggle]);

  const startProgressiveSteer = useCallback((direction: 'left' | 'right') => {
    // Clear any existing interval
    if (steerIntervalRef.current) clearInterval(steerIntervalRef.current);
    setSteeringDirection(direction);
    triggerHaptic('light');

    const sign = direction === 'left' ? -1 : 1;
    // Immediate first step
    const firstAngle = Math.max(-90, Math.min(90, internalAngleRef.current + sign * STEER_INCREMENT));
    internalAngleRef.current = firstAngle;
    onSteeringChange?.(firstAngle);

    // Continue ramping while held
    steerIntervalRef.current = setInterval(() => {
      const next = Math.max(-90, Math.min(90, internalAngleRef.current + sign * STEER_INCREMENT));
      if (next !== internalAngleRef.current) {
        internalAngleRef.current = next;
        onSteeringChange?.(next);
        triggerHaptic('light');
      }
    }, STEER_INTERVAL_MS);
  }, [triggerHaptic, onSteeringChange]);

  const stopProgressiveSteer = useCallback(() => {
    if (steerIntervalRef.current) {
      clearInterval(steerIntervalRef.current);
      steerIntervalRef.current = null;
    }
    setSteeringDirection(null);
    internalAngleRef.current = 0;
    onSteeringChange?.(0);
  }, [onSteeringChange]);

  const handleSteerLeft = useCallback(() => {
    startProgressiveSteer('left');
  }, [startProgressiveSteer]);

  const handleSteerRight = useCallback(() => {
    startProgressiveSteer('right');
  }, [startProgressiveSteer]);

  const handleSteerEnd = useCallback(() => {
    stopProgressiveSteer();
  }, [stopProgressiveSteer]);

  const handleGearChange = useCallback((newGear: string) => {
    triggerHaptic('light');
    onGearChange?.(newGear);
  }, [triggerHaptic, onGearChange]);

  // --- Multitouch-safe touch tracking for all HUD interactive zones ---
  // Each zone tracks its own finger independently, allowing simultaneous
  // steer + throttle, brake + steer, etc.

  useTouchTracking(steerLeftRef, {
    onTouchStart: () => {
      startProgressiveSteer('left');
    },
    onTouchEnd: () => {
      stopProgressiveSteer();
    },
  });

  useTouchTracking(steerRightRef, {
    onTouchStart: () => {
      startProgressiveSteer('right');
    },
    onTouchEnd: () => {
      stopProgressiveSteer();
    },
  });

  useTouchTracking(hudBrakeRef, {
    onTouchStart: () => {
      triggerHaptic('heavy');
      onBrakeChange(true);
    },
    onTouchEnd: () => {
      onBrakeChange(false);
    },
  });

  useTouchTracking(hudThrottleRef, {
    onTouchStart: () => {
      triggerHaptic('medium');
      onThrottleChange(true);
    },
    onTouchEnd: () => {
      onThrottleChange(false);
    },
  });

  if (!isOpen) return null;

  const showStream = isCameraEnabled && isConnected && !!effectiveViewerUrl;

  return (
    <div className="fixed inset-0 z-50">
      {/* Emergency border flash */}
      {(isEmergencyStop || eBrakeActive) && (
        <div className="absolute inset-0 border-4 border-destructive z-50 pointer-events-none animate-pulse" />
      )}
      
      {/* Background Layer - Camera Feed */}
      <div className="absolute inset-0 z-0 bg-background">
        {/* Camera Disabled - Show Off State */}
        {!isCameraEnabled ? (
          <div className="w-full h-full flex items-center justify-center bg-gradient-to-b from-secondary to-background">
            <div className="text-center flex flex-col items-center gap-4">
              <VideoOff className="w-20 h-20 text-muted-foreground/50" />
              <div>
                <div className="text-3xl racing-text text-muted-foreground mb-3">CAMERA DISABLED</div>
                <button
                  onClick={onToggleCamera}
                  className="px-6 py-3 text-base bg-primary/80 hover:bg-primary text-primary-foreground rounded border border-primary/50 transition-colors racing-text font-medium"
                >
                  TURN ON CAMERA
                </button>
              </div>
            </div>
          </div>
        ) : effectiveViewerUrl ? (
          <>
            {/* Iris lens animation overlay */}
            <IrisOverlay phase={irisPhase} />

            {showStream && (
              <iframe
                key={feedViewerKey}
                src={effectiveViewerUrl}
                title="Live Camera Feed"
                className="w-full h-full border-0 pointer-events-none"
                allow="autoplay; picture-in-picture"
                sandbox="allow-scripts allow-same-origin allow-forms allow-pointer-lock"
              />
            )}
            {!showStream && (
              <div className="absolute inset-0 flex flex-col items-center justify-center bg-black/25">
                <VideoOff className="w-10 h-10 text-primary animate-pulse" />
                <div className="mt-2 text-sm racing-text text-muted-foreground">
                  CONNECTING...
                </div>
              </div>
            )}
            {/* Subtle vignette overlay for better HUD readability */}
            <div 
              className="absolute inset-0"
              style={{
                background: 'radial-gradient(ellipse at center, transparent 50%, rgba(0,0,0,0.4) 100%)',
              }}
            />
          </>
        ) : (
          <div className="w-full h-full bg-gradient-to-b from-secondary to-background flex items-center justify-center">
            <div className="text-center">
              <VideoOff className="w-12 h-12 text-muted-foreground/50 mx-auto mb-2" />
              <div className="text-2xl racing-text text-muted-foreground mb-2">NO VIDEO FEED</div>
              <div className="text-sm text-muted-foreground/50">Camera not connected</div>
            </div>
          </div>
        )}
      </div>
      
      {/* HUD Layer */}
      <div className="absolute inset-0 z-10 pointer-events-none">
        
         {/* Steering Zones - Left and Right tap areas (hidden in view-only mode) */}
         {!viewOnly && (
         <div className="absolute inset-0 flex pointer-events-auto">
           {/* Left Steering Zone */}
           <button
             ref={steerLeftRef}
             onMouseDown={handleSteerLeft}
             onMouseUp={handleSteerEnd}
             onMouseLeave={handleSteerEnd}
             className={`flex-1 h-full flex items-center justify-start pl-8 transition-all touch-none select-none ${
               steeringDirection === 'left' ? 'bg-primary/10' : ''
             }`}
           >
             {/* Left indicator */}
             <div className={`flex flex-col items-center gap-1 transition-all ${
               steeringDirection === 'left' ? 'opacity-100 scale-110' : 'opacity-30'
             }`}>
               <div className="flex items-center gap-2">
                 <svg className="w-8 h-8 text-primary" viewBox="0 0 24 24" fill="currentColor">
                   <path d="M15 19l-7-7 7-7" stroke="currentColor" strokeWidth="2" fill="none" strokeLinecap="round" strokeLinejoin="round"/>
                 </svg>
                 <span className="text-xs racing-text text-primary">LEFT</span>
               </div>
               {steeringAngle < 0 && (
                 <span className="text-[10px] racing-number text-primary font-bold">{Math.abs(steeringAngle)}°</span>
               )}
             </div>
           </button>
           
           {/* Center dead zone - no steering */}
           <div className="w-1/3 h-full pointer-events-none" />
           
           {/* Right Steering Zone */}
           <button
             ref={steerRightRef}
             onMouseDown={handleSteerRight}
             onMouseUp={handleSteerEnd}
             onMouseLeave={handleSteerEnd}
             className={`flex-1 h-full flex items-center justify-end pr-8 transition-all touch-none select-none ${
               steeringDirection === 'right' ? 'bg-primary/10' : ''
             }`}
           >
             {/* Right indicator */}
             <div className={`flex flex-col items-center gap-1 transition-all ${
               steeringDirection === 'right' ? 'opacity-100 scale-110' : 'opacity-30'
             }`}>
               <div className="flex items-center gap-2">
                 <span className="text-xs racing-text text-primary">RIGHT</span>
                 <svg className="w-8 h-8 text-primary" viewBox="0 0 24 24" fill="currentColor">
                   <path d="M9 5l7 7-7 7" stroke="currentColor" strokeWidth="2" fill="none" strokeLinecap="round" strokeLinejoin="round"/>
                 </svg>
               </div>
               {steeringAngle > 0 && (
                 <span className="text-[10px] racing-number text-primary font-bold">{Math.abs(steeringAngle)}°</span>
               )}
             </div>
           </button>
         </div>
         )}
         
        {/* Top Center - Status Bar */}
         <div className="absolute top-2 left-1/2 -translate-x-1/2 pointer-events-auto z-20">
          <div className="flex items-center gap-3 px-4 py-1.5 rounded-full bg-background/40 backdrop-blur-md border border-border/50">
            {/* Auto Pilot Toggle (hidden in view-only) */}
            {!viewOnly && (
            <button
              onClick={onAutoModeToggle}
              className={`flex items-center gap-1 px-2 py-0.5 rounded text-[10px] racing-text transition-colors ${
                isAutoMode 
                  ? 'bg-primary text-primary-foreground' 
                  : 'bg-muted/50 text-muted-foreground'
              }`}
            >
              <Zap className="w-3 h-3" />
              AUTO
            </button>
            )}
            
            {/* Connection Status */}
            <div className="flex items-center gap-1">
              <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-destructive'}`} />
              <Wifi className={`w-3 h-3 ${isConnected ? 'text-green-500' : 'text-destructive'}`} />
            </div>
            
            {/* Battery */}
            <div className="flex items-center gap-1 text-[10px] racing-text text-foreground">
              <Power className="w-3 h-3 text-primary" />
              7.4v
            </div>

            {/* Gamepad Status (when in console mode) */}
            {inputMode === "console" && (
              <div className="flex items-center gap-1 text-[10px] racing-text" title="gamepad controller mode">
                <Gamepad2 className={`w-3 h-3 ${gamepadConnected ? 'text-green-400' : 'text-destructive'}`} />
              </div>
            )}
            
            {/* Close Button */}
            <button
              onClick={onClose}
              className="ml-2 p-1 rounded-full bg-muted/50 hover:bg-muted text-foreground"
            >
              <X className="w-4 h-4" />
            </button>
          </div>
        </div>
        
        {/* Top Left - RPM Gauge */}
         <div className="absolute top-4 left-4 z-20 pointer-events-none">
          {/* Camera Info Badge */}
          {isCameraEnabled && (
            <div className="mb-2 flex items-center gap-1.5 px-2.5 py-1 rounded-full bg-background/40 backdrop-blur-md border border-border/50 pointer-events-none">
              <Camera className="w-3 h-3 text-primary flex-shrink-0" />
              <span className="text-[9px] racing-text text-foreground/90 whitespace-nowrap">
                {resolutionLabel}
              </span>
              <span className="text-[9px] text-muted-foreground/60">|</span>
              <span className="text-[9px] racing-text text-foreground/90">
                {cameraActualFps ? Math.round(cameraActualFps) : (cameraFramerate || 30)}fps
              </span>
              <span className="text-[9px] text-muted-foreground/60">|</span>
              <span className="text-[9px] racing-text text-foreground/90">
                Q{cameraJpegQuality || 70}
              </span>
              <span className="text-[9px] text-muted-foreground/60">|</span>
              <div className="flex items-center gap-1">
                <div className={`w-1.5 h-1.5 rounded-full ${userWantsVision ? 'bg-green-500 shadow-[0_0_4px_rgba(34,197,94,0.6)]' : 'bg-muted-foreground/40'}`} />
                <span className={`text-[9px] racing-text ${userWantsVision ? 'text-green-400' : 'text-muted-foreground/60'}`}>
                  CV{visionActive && visionFps ? ` ${Math.round(visionFps)}` : ''}
                </span>
              </div>
            </div>
          )}
          <div className="flex items-center gap-2">
            <div className="w-24 h-24 rounded-full bg-background/40 backdrop-blur-md border border-border/50 flex flex-col items-center justify-center">
              <svg viewBox="0 0 100 100" className={`w-20 h-20 ${isRedline ? 'animate-[shake_0.1s_infinite]' : ''}`}>
                {/* Background arc */}
                <path
                  d="M 15 72 A 40 40 0 1 1 85 72"
                  fill="none"
                  stroke="hsl(var(--muted))"
                  strokeWidth="6"
                  strokeLinecap="round"
                />
                {/* Active arc */}
                <path
                  d="M 15 72 A 40 40 0 1 1 85 72"
                  fill="none"
                  stroke={isRedline ? "hsl(var(--destructive))" : "hsl(var(--primary))"}
                  strokeWidth="6"
                  strokeLinecap="round"
                  strokeDasharray="220"
                  strokeDashoffset={220 - (220 * rpm) / 100}
                  style={{
                    filter: rpm > 50 ? `drop-shadow(0 0 6px ${isRedline ? 'hsl(var(--destructive))' : 'hsl(var(--primary))'})` : 'none',
                    transition: 'stroke-dashoffset 0.1s ease-out'
                  }}
                />
                <text x="50" y="55" textAnchor="middle" fill="hsl(var(--foreground))" fontSize="16" fontWeight="bold">
                  {Math.round(rpm)}%
                </text>
              </svg>
              <div className="text-[10px] racing-text text-primary -mt-2">RPM</div>
            </div>
            
            {/* E-Brake Toggle */}
            {!viewOnly && (
            <button
              onClick={handleEBrake}
              className={`pointer-events-auto px-3 py-1.5 rounded text-xs racing-text border transition-all ${
                eBrakeActive
                  ? 'bg-destructive text-destructive-foreground border-destructive glow-red'
                  : 'bg-background/40 backdrop-blur-md text-muted-foreground border-border/50 hover:border-destructive'
              }`}
            >
              E-BRAKE {eBrakeActive ? 'ON' : 'OFF'}
            </button>
            )}
          </div>
        </div>
        
        {/* Gear Shifter Section - Compact, beside speedometer (hidden in view-only) */}
        {!viewOnly && (
        <div className="absolute top-4 right-40 z-20 pointer-events-auto">
          <div className="bg-background/40 backdrop-blur-md border border-border/50 rounded-lg p-2">
            <div className="text-[8px] racing-text text-muted-foreground mb-1 text-center">GEAR</div>
            <div className="grid grid-cols-3 gap-0.5">
              <button
                onClick={() => handleGearChange('S')}
                className={`w-5 h-5 rounded text-[10px] font-bold racing-text transition-all ${
                  gear === 'S'
                    ? 'bg-primary text-primary-foreground glow-teal'
                    : 'bg-muted/50 text-muted-foreground hover:bg-muted'
                }`}
              >
                S
              </button>
              <button
                onClick={() => handleGearChange('3')}
                className={`w-5 h-5 rounded text-[10px] font-bold racing-text transition-all ${
                  gear === '3'
                    ? 'bg-primary text-primary-foreground glow-teal'
                    : 'bg-muted/50 text-muted-foreground hover:bg-muted'
                }`}
              >
                3
              </button>
              <button
                onClick={() => handleGearChange('2')}
                className={`w-5 h-5 rounded text-[10px] font-bold racing-text transition-all ${
                  gear === '2'
                    ? 'bg-primary text-primary-foreground glow-teal'
                    : 'bg-muted/50 text-muted-foreground hover:bg-muted'
                }`}
              >
                2
              </button>
              <button
                onClick={() => handleGearChange('1')}
                className={`w-5 h-5 rounded text-[10px] font-bold racing-text transition-all ${
                  gear === '1'
                    ? 'bg-primary text-primary-foreground glow-teal'
                    : 'bg-muted/50 text-muted-foreground hover:bg-muted'
                }`}
              >
                1
              </button>
              <button
                onClick={() => handleGearChange('N')}
                className={`w-5 h-5 rounded text-[10px] font-bold racing-text transition-all ${
                  gear === 'N'
                    ? 'bg-primary text-primary-foreground glow-teal'
                    : 'bg-muted/50 text-muted-foreground hover:bg-muted'
                }`}
              >
                N
              </button>
              <button
                onClick={() => handleGearChange('R')}
                className={`w-5 h-5 rounded text-[10px] font-bold racing-text transition-all ${
                  gear === 'R'
                    ? 'bg-destructive text-destructive-foreground glow-red'
                    : 'bg-muted/50 text-muted-foreground hover:bg-muted'
                }`}
              >
                R
              </button>
            </div>
          </div>
        </div>
        )}

        
        {/* Top Right - Speed & Gear */}
         <div className="absolute top-4 right-4 z-20 pointer-events-none">
          <div className="flex items-center gap-2 px-4 py-3 rounded-lg bg-background/40 backdrop-blur-md border border-border/50">
            <div className="text-right">
              <div className="text-3xl racing-number font-bold text-foreground text-glow-teal">
                {hudDisplaySpeed < 10 ? hudDisplaySpeed.toFixed(1) : Math.round(hudDisplaySpeed)}
              </div>
              <div className="text-[8px] racing-text text-muted-foreground">{hudSpeedLabel}</div>
            </div>
            <div className={`w-12 h-12 rounded-lg flex items-center justify-center text-xl font-bold racing-text ${
              gear === 'R' 
                ? 'bg-destructive text-destructive-foreground' 
                : 'bg-primary text-primary-foreground'
            }`}>
              {gear}
            </div>
          </div>
        </div>
        
        {/* Bottom Left - Brake Zone (hidden in view-only) */}
        {!viewOnly && (
         <div className="absolute bottom-4 left-4 pointer-events-auto z-20">
          {/* Brake Pedal */}
          <button
            ref={hudBrakeRef}
            onMouseDown={handleBrakeStart}
            onMouseUp={handleBrakeEnd}
            onMouseLeave={handleBrakeEnd}
            className={`w-32 h-32 rounded-2xl flex flex-col items-center justify-center transition-all touch-none select-none ${
              brake
                ? 'bg-destructive/80 backdrop-blur-md border-2 border-destructive glow-red scale-95'
                : 'bg-destructive/30 backdrop-blur-md border border-destructive/50 hover:bg-destructive/40'
            }`}
          >
            <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <rect x="6" y="4" width="12" height="16" rx="2" />
              <line x1="6" y1="8" x2="18" y2="8" />
            </svg>
            <span className="text-xs racing-text mt-1">BRAKE</span>
          </button>
        </div>
        )}
        
        {/* Bottom Right - Throttle Zone (hidden in view-only) */}
        {!viewOnly && (
         <div className="absolute bottom-4 right-4 pointer-events-auto z-20">
          <button
            ref={hudThrottleRef}
            onMouseDown={handleThrottleStart}
            onMouseUp={handleThrottleEnd}
            onMouseLeave={handleThrottleEnd}
            className={`w-32 h-32 rounded-2xl flex flex-col items-center justify-center transition-all touch-none select-none ${
              throttle
                ? 'bg-primary/80 backdrop-blur-md border-2 border-primary glow-teal scale-95'
                : 'bg-primary/30 backdrop-blur-md border border-primary/50 hover:bg-primary/40'
            }`}
            style={{
              filter: throttle ? 'brightness(1.3)' : 'none'
            }}
          >
            <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M7 20L7 4" />
              <path d="M17 20L17 4" />
              <rect x="4" y="8" width="16" height="8" rx="2" />
            </svg>
            <span className="text-xs racing-text mt-1">THROTTLE</span>
          </button>
        </div>
        )}
        
        {/* AI Narration Overlay */}
        {narrationEnabled && (
          <div className="absolute bottom-40 left-1/2 -translate-x-1/2 z-30 pointer-events-none flex flex-col items-center gap-2 max-w-[80vw]">
            {/* AI badge */}
            <div className={`flex items-center gap-1 px-2 py-0.5 rounded-full backdrop-blur-md border ${
              narrationSpeaking 
                ? 'bg-green-500/20 border-green-500/50' 
                : 'bg-background/30 border-border/50'
            }`}>
              <Mic className={`w-3 h-3 ${narrationSpeaking ? 'text-green-400 animate-pulse' : 'text-primary/70'}`} />
              <span className={`text-[9px] racing-text ${narrationSpeaking ? 'text-green-400' : 'text-muted-foreground'}`}>
                {narrationSpeaking ? 'AI SPEAKING' : 'AI ON'}
              </span>
            </div>
            {/* Narration text with auto-fade */}
            {narrationVisible && narrationLastText && (
              <div 
                className={`px-4 py-2 rounded-lg bg-background/60 backdrop-blur-md border border-border/50 transition-opacity duration-[3000ms] ease-out ${
                  narrationFading ? 'opacity-0' : 'opacity-100'
                }`}
              >
                <p className="text-[11px] sm:text-sm text-foreground racing-text text-center leading-relaxed">
                  🤖 {narrationLastText}
                </p>
              </div>
            )}
          </div>
        )}
        
      </div>
      
      {/* Shake animation for redline */}
      <style>{`
        @keyframes shake {
          0%, 100% { transform: translateX(0); }
          25% { transform: translateX(-2px); }
          75% { transform: translateX(2px); }
        }
      `}</style>
    </div>
  );
};
