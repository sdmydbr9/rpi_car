import { useState, useCallback, useRef, useEffect } from "react";
import { Header } from "./Header";
import { SteeringWheel } from "./SteeringWheel";
import { CameraFeed } from "./CameraFeed";
import { CarTelemetry } from "./CarTelemetry";
import { GearShifter } from "./GearShifter";
import { Pedals } from "./Pedals";
import { ImmersiveHUD } from "../ImmersiveHUD";
import * as socketClient from "../../lib/socketClient";
import { useAutoAcceleration } from "../../hooks/useAutoAcceleration";

interface ControlState {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
}

export const CockpitController = () => {
  const [controlState, setControlState] = useState<ControlState>({
    steeringAngle: 0,
    throttle: false,
    brake: false,
    gear: "N",
    speed: 0,
  });
  const [isConnected, setIsConnected] = useState(false);
  const [serverIp, setServerIp] = useState("");
  const [isEmergencyStop, setIsEmergencyStop] = useState(false);
  const [isAutoMode, setIsAutoMode] = useState(false);
  const [isIREnabled, setIsIREnabled] = useState(true);
  const [isImmersiveView, setIsImmersiveView] = useState(false);
  const [streamUrl, setStreamUrl] = useState<string>("");
  const [isEngineRunning, setIsEngineRunning] = useState(false);
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
        // DO NOT update throttle and brake from telemetry - these are user-controlled inputs
        // that must maintain their state while held
      }));
      // Update IR enabled state from telemetry
      setIsIREnabled(data.ir_enabled ?? true);
    });

    return () => {
      socketClient.onTelemetry(() => {}); // Unsubscribe
    };
  }, [isConnected]);



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
    
    if (newEmergencyStopState) {
      // Activating emergency stop
      console.log('🏁 Emergency stop ACTIVE');
      setIsAutoMode(false);
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
      socketClient.emitEmergencyStop();
    } else {
      // Deactivating emergency stop
      console.log('🏁 Emergency stop RELEASED');
      socketClient.emitEmergencyStopRelease();
    }
  }, [isEmergencyStop]);

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
    socketClient.emitIRToggle();
  }, []);

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
    setControlState(prev => ({ ...prev, throttle: false, brake: false, speed: 0 }));
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
        onThrottleChange={handleThrottleChange}
        onBrakeChange={handleBrakeChange}
        onEmergencyStop={handleEmergencyStop}
        onAutoModeToggle={handleAutoMode}
        onSteeringChange={handleAngleChange}
        onGearChange={handleGearChange}
      />
      
      <div className="h-[100dvh] w-full flex flex-col overflow-hidden">
        {/* Header */}
        <Header 
          isConnected={isConnected}
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
              onLaunch={handleLaunch}
              onDonut={handleDonut}
              isEngineRunning={isEngineRunning}
            />
          </div>
          
          {/* Right Zone: Gear Shifter */}
          <div className="flex-[0.25] border-l border-border/30 racing-panel m-0.5 overflow-hidden">
            <GearShifter 
              currentGear={controlState.gear} 
              onGearChange={handleGearChange}
              isEmergencyStop={isEmergencyStop}
              isAutoMode={isAutoMode}
              isIREnabled={isIREnabled}
              onEmergencyStop={handleEmergencyStop}
              onAutoMode={handleAutoMode}
              onIRToggle={handleIRToggle}
              isEnabled={isEngineRunning}
              isEngineRunning={isEngineRunning}
              onEngineStart={handleEngineStart}
              onEngineStop={handleEngineStop}
            />
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
