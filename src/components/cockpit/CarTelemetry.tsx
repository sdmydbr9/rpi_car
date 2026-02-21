import { useState } from "react";

import { Rocket, RotateCcw } from "lucide-react";
import { ServiceLight, type SensorStatus } from "./ServiceLight";
import { Speedometer, type SpeedUnit } from "./Speedometer";
import { Gauge } from "./Gauge";
import { BatteryGauge } from "./BatteryGauge";
import { AccelerometerHUD } from "./AccelerometerHUD";

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
  encoderAvailable?: boolean; // Whether real wheel encoder is providing RPM
  onLaunch: () => void;
  onDonut: () => void;
  isEngineRunning?: boolean;
  sensors?: SensorStatus[];
  requiresService?: boolean;
  accelX?: number;
  accelY?: number;
  accelZ?: number;
}

export const CarTelemetry = ({
  steeringAngle,
  throttle,
  brake,
  gear,
  speed,
  speedMpm,
  speedUnit = "m/min",
  temperature,
  cpuClock,
  gpuClock,
  batteryVoltage,
  rpm,
  encoderAvailable = false,
  onLaunch,
  onDonut,
  isEngineRunning = false,
  sensors = [],
  requiresService = false,
  accelX = 0,
  accelY = 0,
  accelZ = 0,
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
    <div className="flex flex-col items-center justify-center h-full p-0.5 overflow-auto gap-1" data-scrollable="true" style={{ touchAction: 'pan-y' }}>
      {/* HUD Row: MPU6050 Accelerometer */}
      <div className="w-full flex justify-center mb-0.5">
        <AccelerometerHUD x={accelX} y={accelY} z={accelZ} />
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
              className="transition-all"
              fill="hsl(var(--card))"
              stroke={throttle || brake ? "rgb(20, 184, 166)" : "rgb(107, 114, 128)"}
              strokeWidth="1"
              style={{
                filter: throttle || brake ? 'drop-shadow(0 0 4px rgb(20, 184, 166))' : 'none'
              }}
            />
            
            {/* Rear Right Tire - with heat glow */}
            <rect
              x="78"
              y="115"
              width="14"
              height="28"
              rx="2"
              className="transition-all"
              fill="hsl(var(--card))"
              stroke={throttle || brake ? "rgb(20, 184, 166)" : "rgb(107, 114, 128)"}
              strokeWidth="1"
              style={{
                filter: throttle || brake ? 'drop-shadow(0 0 4px rgb(20, 184, 166))' : 'none'
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

        {/* Right: RPM Gauge - Larger */}
        <div className="flex flex-col items-center">
          <Gauge 
            value={rpm}
            min={0}
            max={300}
            label={encoderAvailable ? "RPM" : "RPM (est.)"}
            unit=""
            isEngineRunning={isEngineRunning}
            warningThreshold={250}
            size="medium"
          />
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
      </div>
    </div>
  );
};
