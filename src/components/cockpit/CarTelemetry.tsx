import { useState } from "react";
import { Rocket, RotateCcw, Radar, Navigation, AlertTriangle } from "lucide-react";
import { Speedometer } from "./Speedometer";
import { Gauge } from "./Gauge";

interface CarTelemetryProps {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
  temperature: number;
  cpuClock: number;
  gpuClock: number;
  rpm: number;
  onLaunch: () => void;
  onDonut: () => void;
  isEngineRunning?: boolean;
  isAutopilotEnabled?: boolean;
  autonomousState?: string;
  sonarDistance?: number;
  autonomousTargetSpeed?: number;
}

export const CarTelemetry = ({
  steeringAngle,
  throttle,
  brake,
  gear,
  speed,
  temperature,
  cpuClock,
  gpuClock,
  rpm,
  onLaunch,
  onDonut,
  isEngineRunning = false,
  isAutopilotEnabled = false,
  autonomousState = "CRUISING",
  sonarDistance = 100,
  autonomousTargetSpeed = 0,
}: CarTelemetryProps) => {
  const [launchActive, setLaunchActive] = useState(false);
  const [donutActive, setDonutActive] = useState(false);

  const handleLaunch = () => {
    setLaunchActive(true);
    onLaunch();
    setTimeout(() => setLaunchActive(false), 500);
  };

  const handleDonut = () => {
    setDonutActive(true);
    onDonut();
    setTimeout(() => setDonutActive(false), 500);
  };

  return (
    <div className="flex flex-col items-center justify-center h-full p-0.5 overflow-auto gap-1">
      {/* First Row: Temperature, CPU, GPU Gauges - Smaller */}
      <div className="flex gap-1 justify-center flex-wrap">
        <Gauge 
          value={temperature}
          min={30}
          max={85}
          label="Temperature"
          unit="°C"
          isEngineRunning={isEngineRunning}
          warningThreshold={75}
          size="small"
        />
        <Gauge 
          value={cpuClock}
          min={600}
          max={2400}
          label="CPU"
          unit="MHz"
          isEngineRunning={isEngineRunning}
          size="small"
        />
        <Gauge 
          value={gpuClock}
          min={150}
          max={500}
          label="GPU"
          unit="MHz"
          isEngineRunning={isEngineRunning}
          size="small"
        />
      </div>

      {/* Second Row: Speedometer, Car Diagram, and RPM Gauge */}
      <div className="flex gap-1 items-start justify-center flex-wrap">
        {/* Left: Speedometer */}
        <div className="flex flex-col items-center">
          <Speedometer speed={speed} maxSpeed={100} isEngineRunning={isEngineRunning} />
        </div>

        {/* Center: Car Diagram with Action Buttons */}
        <div className="relative w-[min(22vw,8rem)]">
          {/* Car Body - Top Down View */}
          <svg viewBox="0 0 100 160" className="w-full h-auto">
            {/* Car Shadow */}
            <ellipse cx="50" cy="85" rx="25" ry="60" fill="hsl(var(--primary) / 0.1)" />
            
            {/* Front Wing */}
            <rect x="15" y="10" width="70" height="8" rx="2" fill="hsl(var(--card))" stroke="hsl(var(--primary) / 0.5)" strokeWidth="0.5" />
            
            {/* Front Left Tire */}
            <g transform={`rotate(${steeringAngle * 0.4}, 22, 28)`}>
              <rect
                x="10"
                y="20"
                width="12"
                height="20"
                rx="2"
                className={`transition-colors ${throttle ? 'fill-muted' : 'fill-card'}`}
                stroke="hsl(var(--border))"
                strokeWidth="1"
              />
            </g>
            
            {/* Front Right Tire */}
            <g transform={`rotate(${steeringAngle * 0.4}, 78, 28)`}>
              <rect
                x="78"
                y="20"
                width="12"
                height="20"
                rx="2"
                className={`transition-colors ${throttle ? 'fill-muted' : 'fill-card'}`}
                stroke="hsl(var(--border))"
                strokeWidth="1"
              />
            </g>
            
            {/* Nose Cone */}
            <path d="M40,15 L50,5 L60,15 L55,35 L45,35 Z" fill="hsl(var(--secondary))" stroke="hsl(var(--primary) / 0.3)" strokeWidth="0.5" />
            
            {/* Cockpit / Main Body */}
            <path 
              d="M30,35 L70,35 L75,70 L78,120 L75,135 L25,135 L22,120 L25,70 Z" 
              fill="hsl(var(--card))"
              stroke="hsl(var(--primary) / 0.3)"
              strokeWidth="1"
            />
            
            {/* Halo */}
            <ellipse cx="50" cy="55" rx="15" ry="10" fill="none" stroke="hsl(var(--muted-foreground))" strokeWidth="3" />
            <circle cx="50" cy="60" r="5" fill="hsl(var(--secondary))" />
            
            {/* Sidepods */}
            <rect x="20" y="50" width="10" height="40" rx="2" fill="hsl(var(--secondary))" stroke="hsl(var(--border))" strokeWidth="0.5" />
            <rect x="70" y="50" width="10" height="40" rx="2" fill="hsl(var(--secondary))" stroke="hsl(var(--border))" strokeWidth="0.5" />
            
            {/* Engine Cover */}
            <rect x="35" y="75" width="30" height="35" rx="3" fill="hsl(var(--muted))" stroke="hsl(var(--border))" strokeWidth="0.5" />
            <line x1="50" y1="80" x2="50" y2="105" stroke="hsl(var(--primary) / 0.5)" strokeWidth="1" />
            
            {/* Rear Left Tire - with heat glow */}
            <rect
              x="8"
              y="115"
              width="14"
              height="28"
              rx="2"
              className={`transition-all ${throttle || brake ? 'fill-destructive/80' : 'fill-card'}`}
              stroke={throttle || brake ? "hsl(var(--destructive))" : "hsl(var(--border))"}
              strokeWidth="1"
              style={{
                filter: throttle || brake ? 'drop-shadow(0 0 4px hsl(var(--destructive)))' : 'none'
              }}
            />
            
            {/* Rear Right Tire - with heat glow */}
            <rect
              x="78"
              y="115"
              width="14"
              height="28"
              rx="2"
              className={`transition-all ${throttle || brake ? 'fill-destructive/80' : 'fill-card'}`}
              stroke={throttle || brake ? "hsl(var(--destructive))" : "hsl(var(--border))"}
              strokeWidth="1"
              style={{
                filter: throttle || brake ? 'drop-shadow(0 0 4px hsl(var(--destructive)))' : 'none'
              }}
            />
            
            {/* Rear Wing */}
            <rect x="12" y="148" width="76" height="6" rx="1" fill="hsl(var(--card))" stroke="hsl(var(--primary) / 0.5)" strokeWidth="0.5" />
            <rect x="20" y="145" width="60" height="3" rx="1" fill="hsl(var(--muted))" />
            
            {/* DRS Indicator */}
            <circle cx="50" cy="150" r="2" className={`transition-colors ${gear === 'S' ? 'fill-primary' : 'fill-muted'}`} />
            
            {/* Petronas Teal Accents */}
            <line x1="30" y1="45" x2="30" y2="130" stroke="hsl(var(--primary))" strokeWidth="1" opacity="0.6" />
            <line x1="70" y1="45" x2="70" y2="130" stroke="hsl(var(--primary))" strokeWidth="1" opacity="0.6" />
          </svg>
          
          {/* Round Action Buttons */}
          <button
            onClick={handleLaunch}
            className={`
              absolute -left-[2.5vw] sm:-left-8 top-1/2 -translate-y-1/2
              w-[8vw] h-[8vw] max-w-10 max-h-10 rounded-full border-2 flex flex-col items-center justify-center
              transition-all duration-100 touch-feedback
              ${launchActive 
                ? 'bg-primary border-primary text-primary-foreground glow-teal scale-95' 
                : 'bg-card border-primary/50 text-primary hover:bg-primary/20'
              }
            `}
          >
            <Rocket className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-none">LAUNCH</span>
          </button>
          
          <button
            onClick={handleDonut}
            className={`
              absolute -right-[2.5vw] sm:-right-8 top-1/2 -translate-y-1/2
              w-[8vw] h-[8vw] max-w-10 max-h-10 rounded-full border-2 flex flex-col items-center justify-center
              transition-all duration-100 touch-feedback
              ${donutActive 
                ? 'bg-accent border-accent text-accent-foreground glow-accent scale-95' 
                : 'bg-card border-accent/50 text-accent hover:bg-accent/20'
              }
            `}
          >
            <RotateCcw className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-none">DONUT</span>
          </button>
        </div>

        {/* Right: RPM Gauge - Larger */}
        <div className="flex flex-col items-center">
          <Gauge 
            value={rpm}
            min={0}
            max={8000}
            label="RPM"
            unit="×100"
            isEngineRunning={isEngineRunning}
            warningThreshold={7500}
            size="medium"
          />
        </div>
      </div>
      
      {/* Status Indicators */}
      <div className="flex gap-2 text-[6px] sm:text-[8px] racing-text">
        <div className={`flex items-center gap-0.5 ${throttle ? 'text-primary text-glow-teal' : 'text-muted-foreground'}`}>
          <div className={`w-1 h-1 sm:w-1.5 sm:h-1.5 rounded-full ${throttle ? 'bg-primary' : 'bg-muted'}`} />
          PWR
        </div>
        <div className={`flex items-center gap-0.5 ${brake ? 'text-destructive text-glow-red' : 'text-muted-foreground'}`}>
          <div className={`w-1 h-1 sm:w-1.5 sm:h-1.5 rounded-full ${brake ? 'bg-destructive' : 'bg-muted'}`} />
          BRK
        </div>
      </div>

      {/* Autonomous Driving Status Panel */}
      {isAutopilotEnabled && (
        <div className="w-full max-w-[20rem] rounded-md border border-primary/40 bg-primary/5 px-2 py-1 flex flex-col gap-0.5">
          {/* Header */}
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-1">
              <Navigation className="w-3 h-3 text-primary animate-pulse" />
              <span className="text-[8px] sm:text-[10px] font-bold racing-text text-primary">
                AUTOPILOT
              </span>
            </div>
            <span className={`text-[7px] sm:text-[9px] font-bold racing-text px-1.5 py-0.5 rounded ${
              autonomousState === 'CRUISING'  ? 'bg-green-500/20 text-green-400' :
              autonomousState === 'BRAKING'   ? 'bg-red-500/20 text-red-400 animate-pulse' :
              autonomousState === 'REVERSING' ? 'bg-amber-500/20 text-amber-400 animate-pulse' :
              autonomousState === 'TURNING'   ? 'bg-blue-500/20 text-blue-400' :
              'bg-muted text-muted-foreground'
            }`}>
              {autonomousState}
            </span>
          </div>
          {/* Sonar bar + target speed */}
          <div className="flex items-center gap-2">
            <Radar className="w-2.5 h-2.5 text-muted-foreground flex-shrink-0" />
            <div className="flex-1 h-1.5 rounded-full bg-muted overflow-hidden">
              <div
                className={`h-full rounded-full transition-all duration-150 ${
                  sonarDistance < 20  ? 'bg-red-500' :
                  sonarDistance < 50  ? 'bg-amber-400' :
                  'bg-green-500'
                }`}
                style={{ width: `${Math.min(100, (sonarDistance / 100) * 100)}%` }}
              />
            </div>
            <span className="text-[7px] sm:text-[9px] racing-text text-muted-foreground w-10 text-right">
              {sonarDistance < 0 ? '--' : `${Math.round(sonarDistance)}cm`}
            </span>
          </div>
          {/* Target speed readout */}
          <div className="flex items-center justify-between text-[6px] sm:text-[8px] racing-text text-muted-foreground">
            <span>TARGET</span>
            <span className="font-bold text-foreground">{autonomousTargetSpeed}%</span>
            {sonarDistance < 20 && (
              <span className="flex items-center gap-0.5 text-red-400">
                <AlertTriangle className="w-2 h-2" />
                DANGER
              </span>
            )}
          </div>
        </div>
      )}
    </div>
  );
};
