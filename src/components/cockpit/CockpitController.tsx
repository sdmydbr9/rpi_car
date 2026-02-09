import { useState, useCallback, useRef, useEffect } from "react";
import { Header } from "./Header";
import { SteeringWheel } from "./SteeringWheel";
import { CameraFeed } from "./CameraFeed";
import { CarTelemetry } from "./CarTelemetry";
import { GearShifter } from "./GearShifter";
import { AutopilotTelemetry } from "./AutopilotTelemetry";
import { Pedals } from "./Pedals";
import { ImmersiveHUD } from "../ImmersiveHUD";
import * as socketClient from "../../lib/socketClient";
import { useAutoAcceleration } from "../../hooks/useAutoAcceleration";
import type { SensorStatus } from "./ServiceLight";
import { DEFAULT_TUNING, type TuningConstants } from "./SettingsDialog";

interface ControlState {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
  temperature: number; // CPU temperature in Celsius
  cpuClock: number; // CPU clock speed in MHz
  gpuClock: number; // GPU clock speed in MHz
  rpm: number; // RPM
}

// Helper function to convert old sensor format to new format
const convertSensorStatus = (
  oldStatus: Record<string, string>
): SensorStatus[] => {
  const sensorNameMap: Record<string, string> = {
    front_sonar: 'Front Sonar',
    rear_sonar: 'Rear Sonar',
    left_ir: 'Left IR',
    right_ir: 'Right IR',
  };

  const statusMap: Record<string, 'ok' | 'warning' | 'error'> = {
    'OK': 'ok',
    'WARNING': 'warning',
    'FAILED': 'error',
  };

  return Object.entries(oldStatus).map(([key, value]) => ({
    name: sensorNameMap[key] || key,
    status: statusMap[value.toUpperCase()] || 'ok',
  }));
};

export const CockpitController = () => {
  const [controlState, setControlState] = useState<ControlState>({
    steeringAngle: 0,
    throttle: false,
    brake: false,
    gear: "N",
    speed: 0,
    temperature: 0,
    cpuClock: 0,
    gpuClock: 0,
    rpm: 0,
  });
  const [isConnected, setIsConnected] = useState(false);
  const [serverIp, setServerIp] = useState("");
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [isAutoMode, setIsAutoMode] = useState(false);
  const [isIREnabled, setIsIREnabled] = useState(true);
  const [isSonarEnabled, setIsSonarEnabled] = useState(true);
  const [isAutopilotEnabled, setIsAutopilotEnabled] = useState(false);
  const [isAutopilotRunning, setIsAutopilotRunning] = useState(false);
  const [isImmersiveView, setIsImmersiveView] = useState(false);
  const [eBrakeActive, setEBrakeActive] = useState(false);
  const [streamUrl, setStreamUrl] = useState<string>("");
  const [isEngineRunning, setIsEngineRunning] = useState(false);
  // Autonomous telemetry state
  const [autonomousState, setAutonomousState] = useState<string>("CRUISING");
  const [sonarDistance, setSonarDistance] = useState<number>(100);
  const [autonomousTargetSpeed, setAutonomousTargetSpeed] = useState<number>(0);
  // Sensor health state
  const [sensors, setSensors] = useState<SensorStatus[]>([
    { name: 'Front Sonar', status: 'ok' },
    { name: 'Rear Sonar', status: 'ok' },
    { name: 'Left IR', status: 'ok' },
    { name: 'Right IR', status: 'ok' },
  ]);
  const [requiresService, setRequiresService] = useState(false);
  const [tuning, setTuning] = useState<TuningConstants>(DEFAULT_TUNING);
  const autoAccelIntervalRef = useRef<number | null>(null);
  const connectionTimeoutRef = useRef<number | null>(null);
  const autoConnectAttemptedRef = useRef(false);

  // Smart auto-connect: try localhost first, then fallback to server IP
  useEffect(() => {
    if (!autoConnectAttemptedRef.current) {
      autoConnectAttemptedRef.current = true;
      const smartAutoConnect = async () => {
        const tryConnection = async (ip: string) => {
          console.log(`🔌 [Startup] Attempting connection to ${ip}:5000...`);
          try {
            await socketClient.connectToServer(ip, 5000);
            setServerIp(ip);
            setIsConnected(true);
            setStreamUrl(`http://${ip}/stream`);
            console.log(`✅ [Startup] Successfully connected to ${ip}`);
            return true;
          } catch (error) {
            console.warn(`⚠️ [Startup] Failed to connect to ${ip}:`, error);
            return false;
          }
        };

        // Try localhost first (for local development)
        const localhostConnected = await tryConnection('localhost');
        
        if (!localhostConnected) {
          // If localhost fails, fetch the server's network IP
          console.log('🔍 [Startup] Localhost failed, fetching server IP...');
          try {
            // Try to get server IP from current origin
            const protocol = window.location.protocol;
            const hostname = window.location.hostname;
            const port = window.location.port;
            const apiUrl = `${protocol}//${hostname}${port ? ':' + port : ''}/api/server-ip`;
            const response = await fetch(apiUrl);
            const data = await response.json();
            console.log(`📡 [Startup] Server IP from API: ${data.ip}`);
            await tryConnection(data.ip);
          } catch (error) {
            console.error('❌ [Startup] Could not fetch server IP:', error);
          }
        }
      };
      smartAutoConnect();
    }
  }, []);

  // Setup telemetry subscription when connected
  useEffect(() => {
    if (!isConnected) {
      return;
    }

    // Subscribe to telemetry updates
    // Note: throttle, brake, and steeringAngle are NOT updated from telemetry - they're controlled only by user input
    // This ensures continuous hold behavior works correctly (the server won't overwrite the UI state)
    socketClient.onTelemetry((data) => {
      setControlState(prev => ({
        ...prev,
        gear: data.gear || prev.gear,
        speed: data.current_pwm || prev.speed,
        // Only show system metrics when engine is running
        temperature: isEngineRunning ? (data.temperature || prev.temperature) : 0,
        cpuClock: isEngineRunning ? (data.cpu_clock || prev.cpuClock) : 0,
        gpuClock: isEngineRunning ? (data.gpu_clock || prev.gpuClock) : 0,
        rpm: isEngineRunning ? (data.rpm || prev.rpm) : 0,
        // DO NOT update throttle and brake from telemetry - these are user-controlled inputs
        // that must maintain their state while held
      }));
      // Update IR enabled state from telemetry
      setIsIREnabled(data.ir_enabled ?? true);
      // Update autonomous telemetry
      if (data.autonomous_mode !== undefined) setIsAutopilotRunning(data.autonomous_mode);
      if (data.autonomous_state) setAutonomousState(data.autonomous_state);
      if (data.sonar_distance !== undefined) setSonarDistance(data.sonar_distance);
      if (data.autonomous_target_speed !== undefined) setAutonomousTargetSpeed(data.autonomous_target_speed);
      if (data.sonar_enabled !== undefined) setIsSonarEnabled(data.sonar_enabled);
      // Update sensor health status
      if (data.sensor_status) {
        const convertedSensors = convertSensorStatus(data.sensor_status);
        setSensors(convertedSensors);
        const hasError = convertedSensors.some(s => s.status !== 'ok');
        setRequiresService(hasError);
      }
    });

    return () => {
      socketClient.onTelemetry(() => {}); // Unsubscribe
    };
  }, [isConnected, isEngineRunning]);



  const handleAngleChange = useCallback((angle: number) => {
    console.log('🎮 Steering angle changed:', angle, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot steer');
      return;
    }
    setControlState(prev => ({ ...prev, steeringAngle: angle }));
    socketClient.emitSteering(Math.round(angle));
  }, [isEngineRunning]);

  const handleThrottleChange = useCallback((active: boolean) => {
    console.log('🎮 Throttle changed:', active, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot throttle');
      return;
    }
    setControlState(prev => ({ ...prev, throttle: active }));
    socketClient.emitThrottle(active);
  }, [isEngineRunning]);

  const handleBrakeChange = useCallback((active: boolean) => {
    console.log('🎮 Brake changed:', active, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot brake');
      return;
    }
    setControlState(prev => ({ ...prev, brake: active }));
    socketClient.emitBrake(active);
  }, [isEngineRunning]);

  const handleGearChange = useCallback((gear: string) => {
    console.log('🎮 Gear changed:', gear, { isEngineRunning, isConnected });
    if (!isEngineRunning) {
      console.warn('⚠️ Engine not running, cannot change gear');
      return;
    }
    setControlState(prev => ({ ...prev, gear }));
    socketClient.emitGearChange(gear);
  }, [isEngineRunning]);

  const handleLaunch = useCallback(() => {
    if (!isEngineRunning || isAutoMode) return;
    // Launch: auto-accelerate in current gear
    setIsAutoMode(true);
    socketClient.emitAutoAccelEnable();
  }, [isEngineRunning, isAutoMode]);

  const handleDonut = useCallback(() => {
    if (!isEngineRunning || isAutoMode) return;
    // Donut: full throttle + steering
    setIsAutoMode(true);
    socketClient.emitThrottle(true);
    socketClient.emitSteering(60);
    socketClient.emitAutoAccelEnable();
  }, [isEngineRunning, isAutoMode]);

  const handleEmergencyStop = useCallback(() => {
    const newEmergencyStopState = !isEmergencyStop;
    console.log('🏁 Emergency stop toggled:', newEmergencyStopState);
    setIsEmergencyStop(newEmergencyStopState);
    setEBrakeActive(newEmergencyStopState);
    
    if (newEmergencyStopState) {
      // Activating emergency stop
      console.log('🏁 Emergency stop ACTIVE');
      setIsAutoMode(false);
      setIsAutopilotEnabled(false);
      setIsAutopilotRunning(false);
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
      socketClient.emitEmergencyStop();
      socketClient.emitAutoAccelDisable();
    } else {
      // Deactivating emergency stop
      console.log('🏁 Emergency stop RELEASED');
      socketClient.emitEmergencyStopRelease();
    }
  }, [isEmergencyStop]);

  // Autopilot-specific e-brake: stops the car but stays in autopilot mode
  const handleAutopilotEBrake = useCallback(() => {
    const newEBrakeState = !eBrakeActive;
    console.log('🛑 Autopilot E-brake toggled:', newEBrakeState);
    setEBrakeActive(newEBrakeState);
    setIsEmergencyStop(newEBrakeState);
    
    if (newEBrakeState) {
      // Activating e-brake while in autopilot - stop car but remain in autopilot mode
      console.log('🛑 Autopilot E-brake ACTIVE - car stopped, autopilot paused');
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false }));
      socketClient.emitEmergencyStop();
    } else {
      // Deactivating e-brake while in autopilot - resume autopilot
      console.log('🛑 Autopilot E-brake RELEASED - resuming autopilot');
      socketClient.emitEmergencyStopRelease();
    }
  }, [eBrakeActive]);

  const handleEBrakeToggle = useCallback(() => {
    const newEBrakeState = !eBrakeActive;
    console.log('🛑 E-brake toggled:', newEBrakeState);
    setEBrakeActive(newEBrakeState);
    
    if (newEBrakeState) {
      // Activating e-brake
      console.log('🛑 E-brake ACTIVE');
      setIsEmergencyStop(true);
      setIsAutopilotEnabled(false);
      setIsAutopilotRunning(false);
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
      socketClient.emitEmergencyStop();
      if (isAutopilotRunning) {
        socketClient.emitAutopilotDisable();
      }
    } else {
      // Deactivating e-brake
      console.log('🛑 E-brake RELEASED');
      setIsEmergencyStop(false);
      socketClient.emitEmergencyStopRelease();
    }
  }, [eBrakeActive, isAutopilotRunning]);

  const handleAutoMode = useCallback(() => {
    if (isEmergencyStop) return; // Cannot enable auto mode during emergency stop
    setIsAutoMode(prev => !prev);
    if (!isAutoMode) {
      socketClient.emitAutoAccelEnable();
    } else {
      socketClient.emitAutoAccelDisable();
    }
  }, [isEmergencyStop, isAutoMode]);

  const handleIRToggle = useCallback(() => {
    console.log('🎮 IR sensor toggle');
    if (isAutopilotRunning) {
      console.log('🚫 IR sensors cannot be toggled in autonomous mode');
      return;
    }
    socketClient.emitIRToggle();
  }, [isAutopilotRunning]);

  const handleSonarToggle = useCallback(() => {
    console.log('🎮 SONAR sensor toggle');
    if (isAutopilotRunning) {
      console.log('🚫 Sonar sensor cannot be toggled in autonomous mode');
      return;
    }
    setIsSonarEnabled(prev => !prev);
    socketClient.emitSonarToggle();
  }, [isAutopilotRunning]);

  // Toggle autopilot VIEW (does not start/stop the autopilot FSM)
  const handleAutopilotToggle = useCallback(() => {
    console.log('🎮 Autopilot view toggle');
    if (eBrakeActive || isEmergencyStop) {
      console.log('🚫 Cannot open autopilot while e-brake/emergency stop is active');
      return;
    }
    const entering = !isAutopilotEnabled;
    if (!entering && isAutopilotRunning) {
      // Exiting autopilot view while running → stop the autopilot first
      socketClient.emitAutopilotDisable();
    }
    setIsAutopilotEnabled(entering);
  }, [eBrakeActive, isEmergencyStop, isAutopilotEnabled, isAutopilotRunning]);

  // Actually start / stop the autopilot FSM on the server
  const handleAutopilotStartStop = useCallback(() => {
    console.log('🎮 Autopilot start/stop, currently running:', isAutopilotRunning);
    if (eBrakeActive || isEmergencyStop) {
      console.log('🚫 Cannot start autopilot while e-brake/emergency stop is active');
      return;
    }
    if (isAutopilotRunning) {
      socketClient.emitAutopilotDisable();
    } else {
      socketClient.emitAutopilotEnable();
    }
  }, [eBrakeActive, isEmergencyStop, isAutopilotRunning]);

  const handleEngineStart = useCallback(() => {
    console.log('🔧 Engine START button clicked');
    setIsEngineRunning(true);
    console.log('✅ Engine started');
  }, []);

  const handleEngineStop = useCallback(() => {
    console.log('🔧 Engine STOP button clicked');
    setIsEngineRunning(false);
    setIsAutoMode(false);
    socketClient.emitAutoAccelDisable();
    socketClient.emitThrottle(false);
    socketClient.emitBrake(true);
    setControlState(prev => ({ ...prev, throttle: false, brake: false, speed: 0, temperature: 0, cpuClock: 0, gpuClock: 0, rpm: 0 }));
    console.log('✅ Engine stopped');
  }, []);

  const handleImmersiveViewToggle = useCallback(() => {
    setIsImmersiveView(prev => !prev);
  }, []);

  // Use auto-acceleration hook for client-side auto-throttle
  useAutoAcceleration({
    enabled: isAutoMode,
    gear: controlState.gear,
    isEngineRunning,
    currentSpeed: controlState.speed,
    isBrakePressed: controlState.brake,
  });

  return (
    <>
      {/* Immersive HUD Overlay */}
      <ImmersiveHUD
        isOpen={isImmersiveView}
        onClose={handleImmersiveViewToggle}
        streamUrl={streamUrl}
        speed={controlState.speed}
        gear={controlState.gear}
        throttle={controlState.throttle}
        brake={controlState.brake}
        isConnected={isConnected}
        isAutoMode={isAutoMode}
        isEmergencyStop={isEmergencyStop}
        eBrakeActive={eBrakeActive}
        onThrottleChange={handleThrottleChange}
        onBrakeChange={handleBrakeChange}
        onEmergencyStop={handleEmergencyStop}
        onEBrakeToggle={handleEBrakeToggle}
        onAutoModeToggle={handleAutoMode}
        onSteeringChange={handleAngleChange}
        onGearChange={handleGearChange}
      />
      
      <div className="h-[100dvh] w-full flex flex-col overflow-hidden">
        {/* Header */}
        <Header 
          isConnected={isConnected}
          tuning={tuning}
          onTuningChange={setTuning}
        />
        
        {/* Main Content - Fixed Layout (No Responsive Changes) */}
        <div className="flex-1 flex min-h-0 overflow-hidden flex-row px-4">
          {/* Left Zone: Camera Feed + Steering Wheel */}
          <div className="flex-[0.35] border-r border-border/30 racing-panel m-0.5 flex flex-col overflow-hidden gap-0.5">
            {/* Camera Feed */}
            <div className="h-[30%] min-h-0 p-0.5 border-none border-b border-border/30">
              <div onClick={handleImmersiveViewToggle} className="cursor-pointer h-full w-full">
                <CameraFeed isConnected={isConnected} />
              </div>
            </div>
            
            {/* Steering Wheel */}
            <div className="flex-1 min-h-0 overflow-hidden flex items-center justify-center">
              <SteeringWheel 
                angle={controlState.steeringAngle} 
                onAngleChange={handleAngleChange}
                isEnabled={isEngineRunning}
              />
            </div>
          </div>
          
          {/* Center Zone: Car Telemetry */}
          <div className="flex-[0.4] racing-panel m-0.5 overflow-hidden">
            <CarTelemetry 
              steeringAngle={controlState.steeringAngle}
              throttle={controlState.throttle}
              brake={controlState.brake}
              gear={controlState.gear}
              speed={controlState.speed}
              temperature={controlState.temperature}
              cpuClock={controlState.cpuClock}
              gpuClock={controlState.gpuClock}
              rpm={controlState.rpm}
              onLaunch={handleLaunch}
              onDonut={handleDonut}
              isEngineRunning={isEngineRunning}
              sensors={sensors}
              requiresService={requiresService}
            />
          </div>
          
          {/* Right Zone: Gear Shifter or Autopilot Telemetry */}
          <div className="flex-[0.25] border-l border-border/30 racing-panel m-0.5 overflow-hidden">
            {isAutopilotEnabled ? (
              <AutopilotTelemetry
                status={autonomousState as any}
                accelerationPercent={autonomousTargetSpeed}
                distanceToObstacle={sonarDistance}
                eBrakeActive={eBrakeActive}
                isRunning={isAutopilotRunning}
                onEmergencyStop={handleAutopilotEBrake}
                onAutopilotToggle={handleAutopilotToggle}
                onStartStop={handleAutopilotStartStop}
              />
            ) : (
              <GearShifter 
                currentGear={controlState.gear} 
                onGearChange={handleGearChange}
                isEmergencyStop={isEmergencyStop}
                isAutoMode={isAutoMode}
                isIREnabled={isIREnabled}
                isSonarEnabled={isSonarEnabled}
                isAutopilotEnabled={isAutopilotEnabled}
                eBrakeActive={eBrakeActive}
                onEmergencyStop={handleEmergencyStop}
                onAutoMode={handleAutoMode}
                onIRToggle={handleIRToggle}
                onSonarToggle={handleSonarToggle}
                onAutopilotToggle={handleAutopilotToggle}
                isEnabled={isEngineRunning}
                isEngineRunning={isEngineRunning}
                onEngineStart={handleEngineStart}
                onEngineStop={handleEngineStop}
              />
            )}
          </div>
        </div>
        
        {/* Footer Zone: Pedals */}
        <div className="h-[12dvh] min-h-12 max-h-20 border-t border-primary/30 flex-shrink-0 px-4">
          <Pedals 
            onThrottleChange={handleThrottleChange}
            onBrakeChange={handleBrakeChange}
            isEnabled={isEngineRunning}
          />
        </div>
      </div>
    </>
  );
};
