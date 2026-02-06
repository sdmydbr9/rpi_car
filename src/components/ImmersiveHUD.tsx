import { useState, useCallback } from "react";
import { X, Wifi, Zap, Power } from "lucide-react";
import { useGameFeedback } from "@/hooks/useGameFeedback";

interface ImmersiveHUDProps {
  isOpen: boolean;
  onClose: () => void;
  streamUrl?: string;
  speed: number;
  gear: string;
  throttle: boolean;
  brake: boolean;
  isConnected: boolean;
  isAutoMode: boolean;
  isEmergencyStop: boolean;
  onThrottleChange: (active: boolean) => void;
  onBrakeChange: (active: boolean) => void;
  onEmergencyStop: () => void;
  onAutoModeToggle: () => void;
  onSteeringChange?: (angle: number) => void;
}

export const ImmersiveHUD = ({
  isOpen,
  onClose,
  streamUrl,
  speed,
  gear,
  throttle,
  brake,
  isConnected,
  isAutoMode,
  isEmergencyStop,
  onThrottleChange,
  onBrakeChange,
  onEmergencyStop,
  onAutoModeToggle,
  onSteeringChange,
}: ImmersiveHUDProps) => {
  const { triggerHaptic, playSound } = useGameFeedback();
  const [eBrakeActive, setEBrakeActive] = useState(false);
  const [steeringDirection, setSteeringDirection] = useState<'left' | 'right' | null>(null);
  
  // RPM simulation based on speed and throttle
  const rpm = Math.min(100, (speed / 100) * 80 + (throttle ? 20 : 0));
  const isRedline = rpm > 85;
  
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
    setEBrakeActive(prev => !prev);
    triggerHaptic('heavy');
    playSound('emergency');
    if (!eBrakeActive) {
      onEmergencyStop();
    }
  }, [eBrakeActive, triggerHaptic, playSound, onEmergencyStop]);

  const handleSteerLeft = useCallback(() => {
    setSteeringDirection('left');
    triggerHaptic('light');
    onSteeringChange?.(-45);
  }, [triggerHaptic, onSteeringChange]);

  const handleSteerRight = useCallback(() => {
    setSteeringDirection('right');
    triggerHaptic('light');
    onSteeringChange?.(45);
  }, [triggerHaptic, onSteeringChange]);

  const handleSteerEnd = useCallback(() => {
    setSteeringDirection(null);
    onSteeringChange?.(0);
  }, [onSteeringChange]);

  if (!isOpen) return null;

  return (
    <div className={`fixed inset-0 z-50 ${isEmergencyStop || eBrakeActive ? 'animate-pulse' : ''}`}>
      {/* Emergency border flash */}
      {(isEmergencyStop || eBrakeActive) && (
        <div className="absolute inset-0 border-4 border-destructive z-50 pointer-events-none animate-pulse" />
      )}
      
      {/* Background Layer - Camera Feed */}
      <div className="absolute inset-0 z-0 bg-background">
        {streamUrl ? (
          <img 
            src={streamUrl} 
            alt="Live Feed" 
            className="w-full h-full object-cover"
          />
        ) : (
          <div className="w-full h-full bg-gradient-to-b from-secondary to-background flex items-center justify-center">
            <div className="text-center">
              <div className="text-4xl racing-text text-muted-foreground mb-2">HELMET CAM</div>
              <div className="text-sm text-muted-foreground/50">Awaiting video feed...</div>
              {/* Simulated track view placeholder */}
              <div className="mt-8 w-64 h-32 mx-auto border border-primary/30 rounded-lg bg-card/30 backdrop-blur-sm flex items-center justify-center">
                <div className="text-xs text-muted-foreground racing-text">TRACK VIEW</div>
              </div>
            </div>
          </div>
        )}
      </div>
      
      {/* HUD Layer */}
      <div className="absolute inset-0 z-10 pointer-events-none">
        
        {/* Steering Zones - Left and Right tap areas */}
        <div className="absolute inset-0 flex pointer-events-auto">
          {/* Left Steering Zone */}
          <button
            onTouchStart={handleSteerLeft}
            onTouchEnd={handleSteerEnd}
            onMouseDown={handleSteerLeft}
            onMouseUp={handleSteerEnd}
            onMouseLeave={handleSteerEnd}
            className={`flex-1 h-full flex items-center justify-start pl-8 transition-all ${
              steeringDirection === 'left' ? 'bg-primary/10' : ''
            }`}
          >
            {/* Left indicator */}
            <div className={`flex items-center gap-2 transition-all ${
              steeringDirection === 'left' ? 'opacity-100 scale-110' : 'opacity-30'
            }`}>
              <svg className="w-8 h-8 text-primary" viewBox="0 0 24 24" fill="currentColor">
                <path d="M15 19l-7-7 7-7" stroke="currentColor" strokeWidth="2" fill="none" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
              <span className="text-xs racing-text text-primary">LEFT</span>
            </div>
          </button>
          
          {/* Center dead zone - no steering */}
          <div className="w-1/3 h-full pointer-events-none" />
          
          {/* Right Steering Zone */}
          <button
            onTouchStart={handleSteerRight}
            onTouchEnd={handleSteerEnd}
            onMouseDown={handleSteerRight}
            onMouseUp={handleSteerEnd}
            onMouseLeave={handleSteerEnd}
            className={`flex-1 h-full flex items-center justify-end pr-8 transition-all ${
              steeringDirection === 'right' ? 'bg-primary/10' : ''
            }`}
          >
            {/* Right indicator */}
            <div className={`flex items-center gap-2 transition-all ${
              steeringDirection === 'right' ? 'opacity-100 scale-110' : 'opacity-30'
            }`}>
              <span className="text-xs racing-text text-primary">RIGHT</span>
              <svg className="w-8 h-8 text-primary" viewBox="0 0 24 24" fill="currentColor">
                <path d="M9 5l7 7-7 7" stroke="currentColor" strokeWidth="2" fill="none" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </div>
          </button>
        </div>
        
        {/* Top Center - Status Bar */}
        <div className="absolute top-2 left-1/2 -translate-x-1/2 pointer-events-auto z-20">
          <div className="flex items-center gap-3 px-4 py-1.5 rounded-full bg-background/40 backdrop-blur-md border border-border/50">
            {/* Auto Pilot Toggle */}
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
        </div>
        
        {/* Top Right - Speed & Gear */}
        <div className="absolute top-4 right-4 z-20 pointer-events-none">
          <div className="flex items-center gap-2 px-4 py-3 rounded-lg bg-background/40 backdrop-blur-md border border-border/50">
            <div className="text-right">
              <div className="text-3xl racing-number font-bold text-foreground text-glow-teal">
                {Math.round(speed)}
              </div>
              <div className="text-[8px] racing-text text-muted-foreground">KM/H</div>
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
        
        {/* Bottom Left - Brake Zone */}
        <div className="absolute bottom-4 left-4 pointer-events-auto z-20">
          <div className="flex flex-col items-start gap-2">
            {/* E-Brake Toggle */}
            <button
              onClick={handleEBrake}
              className={`px-3 py-1.5 rounded text-xs racing-text border transition-all ${
                eBrakeActive
                  ? 'bg-destructive text-destructive-foreground border-destructive glow-red'
                  : 'bg-background/40 backdrop-blur-md text-muted-foreground border-border/50 hover:border-destructive'
              }`}
            >
              E-BRAKE {eBrakeActive ? 'ON' : 'OFF'}
            </button>
            
            {/* Brake Pedal */}
            <button
              onTouchStart={handleBrakeStart}
              onTouchEnd={handleBrakeEnd}
              onMouseDown={handleBrakeStart}
              onMouseUp={handleBrakeEnd}
              onMouseLeave={handleBrakeEnd}
              className={`w-32 h-32 rounded-2xl flex flex-col items-center justify-center transition-all ${
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
        </div>
        
        {/* Bottom Right - Throttle Zone */}
        <div className="absolute bottom-4 right-4 pointer-events-auto z-20">
          <button
            onTouchStart={handleThrottleStart}
            onTouchEnd={handleThrottleEnd}
            onMouseDown={handleThrottleStart}
            onMouseUp={handleThrottleEnd}
            onMouseLeave={handleThrottleEnd}
            className={`w-32 h-32 rounded-2xl flex flex-col items-center justify-center transition-all ${
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
