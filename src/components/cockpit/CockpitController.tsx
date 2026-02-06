import { useState, useCallback, useRef, useEffect } from "react";
import { Header } from "./Header";
import { SteeringWheel } from "./SteeringWheel";
import { CameraFeed } from "./CameraFeed";
import { CarTelemetry } from "./CarTelemetry";
import { GearShifter } from "./GearShifter";
import { Pedals } from "./Pedals";
import { ImmersiveHUD } from "../ImmersiveHUD";

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
  const [isImmersiveView, setIsImmersiveView] = useState(false);
  const [streamUrl, setStreamUrl] = useState<string>("");
  const speedIntervalRef = useRef<number | null>(null);
  const telemetryIntervalRef = useRef<number | null>(null);

  // Simulate speed based on throttle/brake/gear
  useEffect(() => {
    if (speedIntervalRef.current) {
      clearInterval(speedIntervalRef.current);
    }

    speedIntervalRef.current = window.setInterval(() => {
      setControlState(prev => {
        let newSpeed = prev.speed;
        const gearMultiplier = {
          'R': -0.3,
          'N': 0,
          '1': 0.5,
          '2': 0.7,
          '3': 0.9,
          'S': 1.2,
        }[prev.gear] || 0;

        if (prev.throttle && !prev.brake && prev.gear !== 'N') {
          newSpeed = Math.min(100, prev.speed + (2 * gearMultiplier));
        } else if (prev.brake) {
          newSpeed = Math.max(0, prev.speed - 5);
        } else {
          // Natural deceleration
          newSpeed = Math.max(0, prev.speed - 0.5);
        }

        return { ...prev, speed: Math.max(0, newSpeed) };
      });
    }, 100);

    return () => {
      if (speedIntervalRef.current) {
        clearInterval(speedIntervalRef.current);
      }
    };
  }, []);

  // Fetch telemetry data when connected
  useEffect(() => {
    if (!isConnected || !serverIp) {
      if (telemetryIntervalRef.current) {
        clearInterval(telemetryIntervalRef.current);
        telemetryIntervalRef.current = null;
      }
      return;
    }

    telemetryIntervalRef.current = window.setInterval(async () => {
      try {
        const response = await fetch(`http://${serverIp}/telemetry`);
        if (response.ok) {
          const data = await response.json();
          setControlState(prev => ({
            ...prev,
            steeringAngle: data.steer_angle || prev.steeringAngle,
            gear: data.gear || prev.gear,
            speed: data.current_pwm || prev.speed,
            throttle: data.direction === 'forward',
            brake: data.direction === 'stop',
          }));
        }
      } catch (error) {
        console.error("Failed to fetch telemetry:", error);
      }
    }, 500); // Poll every 500ms

    return () => {
      if (telemetryIntervalRef.current) {
        clearInterval(telemetryIntervalRef.current);
        telemetryIntervalRef.current = null;
      }
    };
  }, [isConnected, serverIp]);

  const sendCommand = useCallback((command: string, value: unknown) => {
    if (!isConnected || !serverIp) return;
    
    const baseUrl = `http://${serverIp}`;
    
    try {
      switch (command) {
        case "steering":
          fetch(`${baseUrl}/steer/${value}`);
          break;
        case "throttle":
          if (value) {
            fetch(`${baseUrl}/forward`);
          } else {
            fetch(`${baseUrl}/stop`);
          }
          break;
        case "brake":
          if (value) {
            fetch(`${baseUrl}/brake`);
          }
          break;
        case "gear":
          fetch(`${baseUrl}/gear/${value}`);
          break;
        case "action":
          if (value === "launch") {
            fetch(`${baseUrl}/forward`);
          }
          break;
      }
    } catch (error) {
      console.error(`Failed to send ${command}:`, error);
    }
  }, [isConnected, serverIp]);

  const handleConnect = useCallback((ip: string) => {
    setServerIp(ip);
    setIsConnected(true);
    setStreamUrl(`http://${ip}/stream`);
    console.log("Connected to:", ip);
  }, []);

  const handleDisconnect = useCallback(() => {
    setIsConnected(false);
    setServerIp("");
    setStreamUrl("");
    setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
    if (telemetryIntervalRef.current) {
      clearInterval(telemetryIntervalRef.current);
      telemetryIntervalRef.current = null;
    }
    console.log("Disconnected");
  }, []);

  const handleImmersiveViewToggle = useCallback(() => {
    setIsImmersiveView(prev => !prev);
  }, []);

  const handleAngleChange = useCallback((angle: number) => {
    setControlState(prev => ({ ...prev, steeringAngle: angle }));
    sendCommand("steering", Math.round(angle));
  }, [sendCommand]);

  const handleThrottleChange = useCallback((active: boolean) => {
    setControlState(prev => ({ ...prev, throttle: active }));
    sendCommand("throttle", active);
  }, [sendCommand]);

  const handleBrakeChange = useCallback((active: boolean) => {
    setControlState(prev => ({ ...prev, brake: active }));
    sendCommand("brake", active);
  }, [sendCommand]);

  const handleGearChange = useCallback((gear: string) => {
    setControlState(prev => ({ ...prev, gear }));
    sendCommand("gear", gear);
  }, [sendCommand]);

  const handleLaunch = useCallback(() => {
    sendCommand("action", "launch");
  }, [sendCommand]);

  const handleDonut = useCallback(() => {
    sendCommand("action", "donut");
  }, [sendCommand]);

  const handleEmergencyStop = useCallback(() => {
    setIsEmergencyStop(prev => !prev);
    if (!isEmergencyStop) {
      // Activating emergency stop
      setIsAutoMode(false);
      setControlState(prev => ({ ...prev, speed: 0, throttle: false, brake: false, gear: 'N' }));
      sendCommand("action", "emergency_stop");
    }
  }, [isEmergencyStop, sendCommand]);

  const handleAutoMode = useCallback(() => {
    if (isEmergencyStop) return; // Cannot enable auto mode during emergency stop
    setIsAutoMode(prev => !prev);
  }, [isEmergencyStop]);

  // Auto mode acceleration logic
  useEffect(() => {
    if (!isAutoMode || !isConnected || isEmergencyStop) return;

    const autoAccelInterval = setInterval(() => {
      setControlState(prev => {
        if (prev.gear === 'N') return prev;
        
        let newSpeed = prev.speed;
        const gearMultiplier = {
          'R': -0.3,
          'N': 0,
          '1': 0.5,
          '2': 0.7,
          '3': 0.9,
          'S': 1.2,
        }[prev.gear] || 0;

        if (!isEmergencyStop && prev.gear !== 'N') {
          newSpeed = Math.min(100, prev.speed + (2 * gearMultiplier));
        }

        return { ...prev, speed: Math.max(0, newSpeed), throttle: !isEmergencyStop };
      });
    }, 100);

    return () => clearInterval(autoAccelInterval);
  }, [isAutoMode, isConnected, isEmergencyStop]);

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
          onConnect={handleConnect}
          onDisconnect={handleDisconnect}
        />
        
        {/* Main Content */}
        <div className="flex-1 flex min-h-0 overflow-hidden">
          {/* Left Zone: Camera Feed + Steering Wheel */}
          <div className="flex-[0.35] border-r border-border/30 racing-panel m-0.5 flex flex-col overflow-hidden">
            {/* Camera Feed - Top */}
            <div className="h-[30%] min-h-[4rem] p-0.5 border-b border-border/30">
              <div onClick={handleImmersiveViewToggle} className="cursor-pointer h-full">
                <CameraFeed isConnected={isConnected} />
              </div>
            </div>
            
            {/* Steering Wheel - Bottom */}
            <div className="flex-1 min-h-0 overflow-hidden">
              <SteeringWheel 
                angle={controlState.steeringAngle} 
                onAngleChange={handleAngleChange} 
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
            />
          </div>
          
          {/* Right Zone: Gear Shifter */}
          <div className="flex-[0.25] border-l border-border/30 racing-panel m-0.5 overflow-hidden">
            <GearShifter 
              currentGear={controlState.gear} 
              onGearChange={handleGearChange}
              isEmergencyStop={isEmergencyStop}
              isAutoMode={isAutoMode}
              onEmergencyStop={handleEmergencyStop}
              onAutoMode={handleAutoMode}
            />
          </div>
        </div>
        
        {/* Footer Zone: Pedals */}
        <div className="h-[12dvh] min-h-12 max-h-20 border-t border-primary/30 flex-shrink-0">
          <Pedals 
            onThrottleChange={handleThrottleChange}
            onBrakeChange={handleBrakeChange}
          />
        </div>
      </div>
    </>
  );
};
