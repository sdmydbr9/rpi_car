import { useId, useState } from "react";

import { Rocket, RotateCcw } from "lucide-react";
import { ServiceLight, type SensorStatus } from "./ServiceLight";
import { Speedometer, type SpeedUnit } from "./Speedometer";
import { Gauge } from "./Gauge";
import { BatteryGauge } from "./BatteryGauge";
import { CompassRose } from "./CompassRose";

interface CarTelemetryProps {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
  speedMpm: number;         // Real speed in meters per minute
  speedUnit?: SpeedUnit;    // Display unit for speedometer
  temperature: number;
  cpuClock: number;
  gpuClock: number;
  batteryVoltage: number;
  rpm: number;
  rpmLeft?: number;           // Left encoder RPM
  rpmRight?: number;          // Right encoder RPM
  targetRpm?: number;        // Target RPM from gear system
  encoderAvailable?: boolean; // Whether real wheel encoder is providing RPM
  onLaunch: () => void;
  onDonut: () => void;
  isEngineRunning?: boolean;
  sensors?: SensorStatus[];
  requiresService?: boolean;
  compassHeading?: number;
  compassTargetHeading?: number;
  pidCorrection?: number;
  wheelSync?: {
    status: string;
    active_ticks?: number;
    fallback_count?: number;
    target_rpm?: number;
    actual_rpm?: number;
    gear?: string;
    gear_max_rpm?: number;
    wheels: Record<string, {
      target_rpm: number;
      actual_rpm: number;
      applied_pwm: number;
      error: number;
      has_encoder: boolean;
      on_target?: boolean;
    }>;
  } | null;
}

export const CarTelemetry = ({
  steeringAngle,
  throttle,
  brake,
  gear,
  speedMpm,
  speedUnit = "m/min",
  temperature,
  cpuClock,
  gpuClock,
  batteryVoltage,
  rpm,
  rpmLeft = 0,
  rpmRight = 0,
  targetRpm = 0,
  encoderAvailable = false,
  onLaunch,
  onDonut,
  isEngineRunning = false,
  sensors = [],
  requiresService = false,
  compassHeading = 0,
  compassTargetHeading = 0,
  pidCorrection = 0,
  wheelSync = null,
}: CarTelemetryProps) => {
  const [launchActive, setLaunchActive] = useState(false);
  const [donutActive, setDonutActive] = useState(false);
  const tireAssetId = useId().replace(/:/g, "");
  const wheelSyncWheels = wheelSync?.wheels;

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

  const frontWheelRotation = steeringAngle * 0.4;
  const treadAnimationClass = gear === "R" ? "animate-tread-reverse" : "animate-tread";
  const rearWheelStroke = throttle || brake ? "rgb(20, 184, 166)" : "rgb(107, 114, 128)";
  const rearWheelFilter = throttle || brake ? "drop-shadow(0 0 4px rgb(20, 184, 166))" : "none";
  const tireGradientId = `${tireAssetId}-tire3d`;
  const frontLeftClipId = `${tireAssetId}-tire-front-left`;
  const frontRightClipId = `${tireAssetId}-tire-front-right`;
  const rearLeftClipId = `${tireAssetId}-tire-rear-left`;
  const rearRightClipId = `${tireAssetId}-tire-rear-right`;

  const getWheelRpm = (wheelId: "fl" | "fr" | "rl" | "rr") => {
    const syncedRpm = wheelSyncWheels?.[wheelId]?.actual_rpm;
    if (typeof syncedRpm === "number" && Number.isFinite(syncedRpm)) {
      return Math.abs(syncedRpm);
    }

    const sideRpm = wheelId.endsWith("l") ? rpmLeft : rpmRight;
    if (Number.isFinite(sideRpm) && sideRpm > 0) {
      return Math.abs(sideRpm);
    }

    return Number.isFinite(rpm) && rpm > 0 ? Math.abs(rpm) : 0;
  };

  const getTreadDuration = (wheelRpm: number) => {
    if (wheelRpm <= 0) {
      return "0.6s";
    }

    return `${Math.min(1.5, Math.max(0.08, 60 / wheelRpm))}s`;
  };

  const frontLeftRpm = getWheelRpm("fl");
  const frontRightRpm = getWheelRpm("fr");
  const rearLeftRpm = getWheelRpm("rl");
  const rearRightRpm = getWheelRpm("rr");
  const frontLeftSpinning = frontLeftRpm > 1;
  const frontRightSpinning = frontRightRpm > 1;
  const rearLeftSpinning = rearLeftRpm > 1;
  const rearRightSpinning = rearRightRpm > 1;
  const frontLeftDuration = getTreadDuration(frontLeftRpm);
  const frontRightDuration = getTreadDuration(frontRightRpm);
  const rearLeftDuration = getTreadDuration(rearLeftRpm);
  const rearRightDuration = getTreadDuration(rearRightRpm);

  return (
    <div className="flex flex-col items-center justify-center h-full p-0.5 overflow-auto gap-1" data-scrollable="true" style={{ touchAction: 'pan-y' }}>
      {/* HUD Row: Compass Rose */}
      <div className="w-full flex justify-center mb-0.5">
        <CompassRose heading={compassHeading} targetHeading={compassTargetHeading} pidCorrection={pidCorrection} />
      </div>
      {/* First Row: Temperature, CPU, GPU, Battery Gauges - Smaller */}
      <div className="flex gap-1 justify-center flex-wrap items-end">
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
        <BatteryGauge 
          percentage={Math.max(0, Math.min(100, (batteryVoltage / 13) * 100))}
          voltage={batteryVoltage}
          isEngineRunning={isEngineRunning}
          size="small"
        />
      </div>

      {/* Second Row: Speedometer, Car Diagram, and RPM Gauge */}
      <div className="flex gap-1 items-start justify-center flex-wrap">
        {/* Left: Speedometer */}
        <div className="flex flex-col items-center">
          <Speedometer speedMpm={speedMpm} speedUnit={speedUnit} isEngineRunning={isEngineRunning} />
        </div>

        {/* Center: Car Diagram with Action Buttons */}
        <div className="relative w-[min(22vw,8rem)]">
          {/* Car Body - Top Down View */}
          <svg viewBox="0 0 100 160" className="w-full h-auto">
            <defs>
              <linearGradient id={tireGradientId} x1="0%" y1="0%" x2="100%" y2="0%">
                <stop offset="0%" stopColor="hsl(var(--background) / 0.82)" />
                <stop offset="20%" stopColor="hsl(var(--foreground) / 0.04)" />
                <stop offset="50%" stopColor="hsl(var(--muted-foreground) / 0.16)" />
                <stop offset="80%" stopColor="hsl(var(--foreground) / 0.04)" />
                <stop offset="100%" stopColor="hsl(var(--background) / 0.88)" />
              </linearGradient>
              <clipPath id={frontLeftClipId}>
                <rect x="10" y="20" width="12" height="20" rx="2" />
              </clipPath>
              <clipPath id={frontRightClipId}>
                <rect x="78" y="20" width="12" height="20" rx="2" />
              </clipPath>
              <clipPath id={rearLeftClipId}>
                <rect x="8" y="115" width="14" height="28" rx="2" />
              </clipPath>
              <clipPath id={rearRightClipId}>
                <rect x="78" y="115" width="14" height="28" rx="2" />
              </clipPath>
            </defs>

            {/* Car Shadow */}
            <ellipse cx="50" cy="85" rx="25" ry="60" fill="hsl(var(--primary) / 0.1)" />
            
            {/* Front Wing */}
            <rect x="15" y="10" width="70" height="8" rx="2" fill="hsl(var(--card))" stroke="hsl(var(--primary) / 0.5)" strokeWidth="0.5" />
            
            {/* Front Left Tire */}
            <g transform={`rotate(${frontWheelRotation}, 22, 28)`}>
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
              <rect x="10" y="20" width="12" height="20" rx="2" fill={`url(#${tireGradientId})`} />
              {frontLeftSpinning && (
                <g clipPath={`url(#${frontLeftClipId})`}>
                  <g className={treadAnimationClass} style={{ animationDuration: frontLeftDuration }}>
                    {[-1, 0, 1, 2, 3, 4].map((i) => (
                      <rect
                        key={`fl-${i}`}
                        x="12"
                        y={20 + i * 4}
                        width="8"
                        height="2"
                        rx="0.5"
                        fill="hsl(var(--muted-foreground) / 0.4)"
                      />
                    ))}
                  </g>
                </g>
              )}
            </g>
            
            {/* Front Right Tire */}
            <g transform={`rotate(${frontWheelRotation}, 78, 28)`}>
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
              <rect x="78" y="20" width="12" height="20" rx="2" fill={`url(#${tireGradientId})`} />
              {frontRightSpinning && (
                <g clipPath={`url(#${frontRightClipId})`}>
                  <g className={treadAnimationClass} style={{ animationDuration: frontRightDuration }}>
                    {[-1, 0, 1, 2, 3, 4].map((i) => (
                      <rect
                        key={`fr-${i}`}
                        x="80"
                        y={20 + i * 4}
                        width="8"
                        height="2"
                        rx="0.5"
                        fill="hsl(var(--muted-foreground) / 0.4)"
                      />
                    ))}
                  </g>
                </g>
              )}
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
            <g>
              <rect
                x="8"
                y="115"
                width="14"
                height="28"
                rx="2"
                className="transition-all"
                fill="hsl(var(--card))"
                stroke={rearWheelStroke}
                strokeWidth="1"
                style={{ filter: rearWheelFilter }}
              />
              <rect x="8" y="115" width="14" height="28" rx="2" fill={`url(#${tireGradientId})`} />
              {rearLeftSpinning && (
                <g clipPath={`url(#${rearLeftClipId})`}>
                  <g className={treadAnimationClass} style={{ animationDuration: rearLeftDuration }}>
                    {[-1, 0, 1, 2, 3, 4, 5, 6].map((i) => (
                      <rect
                        key={`rl-${i}`}
                        x="10"
                        y={115 + i * 4}
                        width="10"
                        height="2"
                        rx="0.5"
                        fill="hsl(var(--muted-foreground) / 0.4)"
                      />
                    ))}
                  </g>
                </g>
              )}
            </g>
            
            {/* Rear Right Tire - with heat glow */}
            <g>
              <rect
                x="78"
                y="115"
                width="14"
                height="28"
                rx="2"
                className="transition-all"
                fill="hsl(var(--card))"
                stroke={rearWheelStroke}
                strokeWidth="1"
                style={{ filter: rearWheelFilter }}
              />
              <rect x="78" y="115" width="14" height="28" rx="2" fill={`url(#${tireGradientId})`} />
              {rearRightSpinning && (
                <g clipPath={`url(#${rearRightClipId})`}>
                  <g className={treadAnimationClass} style={{ animationDuration: rearRightDuration }}>
                    {[-1, 0, 1, 2, 3, 4, 5, 6].map((i) => (
                      <rect
                        key={`rr-${i}`}
                        x="80"
                        y={115 + i * 4}
                        width="10"
                        height="2"
                        rx="0.5"
                        fill="hsl(var(--muted-foreground) / 0.4)"
                      />
                    ))}
                  </g>
                </g>
              )}
            </g>
            
            {/* Rear Wing */}
            <rect x="12" y="148" width="76" height="6" rx="1" fill="hsl(var(--card))" stroke="hsl(var(--primary) / 0.5)" strokeWidth="0.5" />
            <rect x="20" y="145" width="60" height="3" rx="1" fill="hsl(var(--muted))" />
            
            {/* DRS Indicator */}
            <circle cx="50" cy="150" r="2" className={`transition-colors ${gear === '4' ? 'fill-primary' : 'fill-muted'}`} />
            
            {/* Petronas Teal Accents */}
            <line x1="30" y1="45" x2="30" y2="130" stroke="hsl(var(--primary))" strokeWidth="1" opacity="0.6" />
            <line x1="70" y1="45" x2="70" y2="130" stroke="hsl(var(--primary))" strokeWidth="1" opacity="0.6" />
          </svg>
          
          {/* Round Action Buttons - Service Button Style */}
          <button
            onClick={handleLaunch}
            className="
              absolute -left-[2.5vw] sm:-left-8 top-1/2 -translate-y-1/2
              w-[8vw] h-[8vw] max-w-10 max-h-10 rounded-full border flex flex-col items-center justify-center
              transition-all duration-100 touch-feedback
            "
            style={{
              outline: 'none',
              border: '1px solid rgb(20, 184, 166)',
              color: launchActive ? 'rgb(20, 184, 166)' : 'rgb(107, 114, 128)',
              backgroundColor: 'transparent'
            }}
          >
            <Rocket className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-none">LAUNCH</span>
          </button>
          
          <button
            onClick={handleDonut}
            className="
              absolute -right-[2.5vw] sm:-right-8 top-1/2 -translate-y-1/2
              w-[8vw] h-[8vw] max-w-10 max-h-10 rounded-full border flex flex-col items-center justify-center
              transition-all duration-100 touch-feedback
            "
            style={{
              outline: 'none',
              border: '1px solid rgb(20, 184, 166)',
              color: donutActive ? 'rgb(20, 184, 166)' : 'rgb(107, 114, 128)',
              backgroundColor: 'transparent'
            }}
          >
            <RotateCcw className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
            <span className="text-[4px] sm:text-[6px] font-bold racing-text leading-none">DONUT</span>
          </button>

          {/* Service Light beside Donut button */}
          <div className="absolute -right-[-10vw] sm:-right-24 top-1/2 -translate-y-1/2">
            <ServiceLight 
              sensors={sensors}
              requiresService={requiresService}
            />
          </div>
        </div>

        {/* Right: Left & Right RPM Gauges */}
        <div className="flex flex-col items-center gap-0.5">
          <div className="flex gap-1">
            <Gauge 
              value={rpmLeft}
              min={0}
              max={500}
              label="L RPM"
              unit=""
              isEngineRunning={isEngineRunning}
              warningThreshold={400}
              size="small"
            />
            <Gauge 
              value={rpmRight}
              min={0}
              max={500}
              label="R RPM"
              unit=""
              isEngineRunning={isEngineRunning}
              warningThreshold={400}
              size="small"
            />
          </div>
          {targetRpm > 0 && (
            <div className="text-[5px] sm:text-[7px] racing-text text-muted-foreground/60">
              Target: {Math.round(targetRpm)} RPM
            </div>
          )}
        </div>
      </div>
      
      {/* Status Indicators */}
      <div className="flex gap-2 text-[6px] sm:text-[8px] racing-text">
        <div className="flex items-center gap-0.5" style={{ color: throttle ? 'rgb(20, 184, 166)' : 'rgb(107, 114, 128)' }}>
          <div className="w-1 h-1 sm:w-1.5 sm:h-1.5 rounded-full" style={{ backgroundColor: throttle ? 'rgb(20, 184, 166)' : 'rgb(107, 114, 128)' }} />
          PWR
        </div>
        <div className="flex items-center gap-0.5" style={{ color: brake ? 'rgb(20, 184, 166)' : 'rgb(107, 114, 128)' }}>
          <div className="w-1 h-1 sm:w-1.5 sm:h-1.5 rounded-full" style={{ backgroundColor: brake ? 'rgb(20, 184, 166)' : 'rgb(107, 114, 128)' }} />
          BRK
        </div>
        {wheelSync && (
          <div className="flex items-center gap-0.5" style={{
            color: wheelSync.status === 'ACTIVE' ? 'rgb(20, 184, 166)'
                 : wheelSync.status === 'FALLBACK' ? 'rgb(234, 179, 8)'
                 : 'rgb(107, 114, 128)'
          }}>
            <div className="w-1 h-1 sm:w-1.5 sm:h-1.5 rounded-full" style={{
              backgroundColor: wheelSync.status === 'ACTIVE' ? 'rgb(20, 184, 166)'
                             : wheelSync.status === 'FALLBACK' ? 'rgb(234, 179, 8)'
                             : 'rgb(107, 114, 128)'
            }} />
            SYNC
          </div>
        )}
      </div>

      {/* Wheel Sync Per-Wheel RPM */}
      {wheelSync && wheelSync.status === 'ACTIVE' && wheelSync.wheels && Object.keys(wheelSync.wheels).length > 0 && (
        <div className="w-full px-1 mt-0.5">
          {wheelSync.gear_max_rpm !== undefined && wheelSync.gear_max_rpm > 0 && (
            <div className="text-center text-[5px] sm:text-[7px] racing-text text-muted-foreground/60 mb-0.5">
              Gear {wheelSync.gear || '?'} &middot; max {wheelSync.gear_max_rpm} RPM
            </div>
          )}
          <div className="grid grid-cols-4 gap-0.5 text-[5px] sm:text-[7px] racing-text text-center">
            {(['fl', 'fr', 'rl', 'rr'] as const).map((wid) => {
              const w = wheelSync.wheels[wid];
              if (!w) return null;
              const onTarget = w.on_target ?? (Math.abs(w.error) <= 10);
              return (
                <div key={wid} className="flex flex-col items-center gap-0">
                  <span className="text-muted-foreground/70 uppercase font-bold">{wid}</span>
                  <span style={{ color: onTarget ? 'rgb(52, 211, 153)' : 'rgb(248, 113, 113)' }}>
                    {w.actual_rpm.toFixed(0)}
                  </span>
                  <span className="text-muted-foreground/50">
                    {w.has_encoder ? `→${w.target_rpm.toFixed(0)}` : 'mirror'}
                  </span>
                </div>
              );
            })}
          </div>
        </div>
      )}
    </div>
  );
};
