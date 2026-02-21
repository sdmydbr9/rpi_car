import { SettingsDialog, TuningConstants, DEFAULT_TUNING } from "./SettingsDialog";
import type { CameraSpecs, NarrationConfig } from "../../lib/socketClient";
import { Volume2 } from "lucide-react";

interface HeaderProps {
  driverName?: string;
  position?: string;
  isConnected: boolean;
  tuning?: TuningConstants;
  onTuningChange?: (tuning: TuningConstants) => void;
  backendDefaults?: TuningConstants;
  cameraSpecs?: CameraSpecs;
  // AI Narration
  narrationConfig?: NarrationConfig;
  imageAnalysisEnabled?: boolean;
  onImageAnalysisToggle?: (enabled: boolean) => void;
  // TTS Audio Unlock
  ttsUnlocked?: boolean;
  onUnlockAudio?: () => void;
  // Sensor toggles
  isIREnabled?: boolean;
  isSonarEnabled?: boolean;
  isMPU6050Enabled?: boolean;
  isRearSonarEnabled?: boolean;
  isCameraEnabled?: boolean;
  onIRToggle?: () => void;
  onSonarToggle?: () => void;
  onRearSonarToggle?: () => void;
  onMPU6050Toggle?: () => void;
  onCameraToggle?: () => void;
  isAutopilotRunning?: boolean;
  onResetDriver?: () => void;
}

export const Header = ({ 
  driverName = "HAMILTON", 
  position = "P1",
  isConnected,
  tuning = DEFAULT_TUNING,
  onTuningChange = () => {},
  backendDefaults = DEFAULT_TUNING,
  cameraSpecs,
  narrationConfig,
  imageAnalysisEnabled,
  onImageAnalysisToggle,
  ttsUnlocked = false,
  onUnlockAudio,
  isIREnabled,
  isSonarEnabled,
  isMPU6050Enabled,
  isRearSonarEnabled,
  isCameraEnabled,
  onIRToggle,
  onSonarToggle,
  onRearSonarToggle,
  onMPU6050Toggle,
  onCameraToggle,
  isAutopilotRunning,
  onResetDriver,
}: HeaderProps) => {
  return (
    <header className="flex items-center justify-between px-1.5 sm:px-4 py-0.5 sm:py-1.5 border-b border-primary/30 bg-card/30 backdrop-blur-sm h-[6dvh] min-h-[1.75rem] max-h-10 flex-shrink-0">
      {/* Left: Team Logo */}
      <div className="flex items-center gap-0.5 sm:gap-2">
        <div className="flex items-center gap-0.5">
          {/* AMG Stripes */}
          <div className="flex gap-px sm:gap-0.5">
            {[...Array(5)].map((_, i) => (
              <div 
                key={i} 
                className="w-0.5 h-2.5 sm:h-4 bg-gradient-to-b from-muted-foreground to-muted transform -skew-x-12"
              />
            ))}
          </div>
          <span className="text-foreground font-bold tracking-wider ml-0.5 text-[10px] sm:text-sm">AMG</span>
        </div>
        <span className="text-primary font-bold racing-text text-[10px] sm:text-sm">PETRONAS</span>
      </div>
      
      {/* Center: Connection Status */}
      <div className="flex items-center gap-0.5 sm:gap-2 text-[8px] sm:text-xs racing-text text-muted-foreground">
        <div className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-primary animate-pulse' : 'bg-destructive'}`} />
        <span className="hidden sm:inline">{isConnected ? 'CONNECTED' : 'OFFLINE'}</span>
      </div>
      
      {/* Right: Driver Status + Settings */}
      <div className="flex items-center gap-1 sm:gap-3">
        <div className="flex items-center gap-0.5 sm:gap-2">
          <span className="text-foreground font-bold racing-text text-[10px] sm:text-sm hidden sm:inline">{driverName}</span>
        </div>
        {/* Unlock Audio Button - shown when image analysis is enabled but audio isn't unlocked */}
        {imageAnalysisEnabled && !ttsUnlocked && (
          <button
            onClick={onUnlockAudio}
            className="px-2 sm:px-3 py-1.5 sm:py-2 rounded-lg border border-amber-500 bg-amber-500/20 text-amber-400 hover:bg-amber-500/30 transition-all touch-feedback flex items-center gap-1.5 animate-pulse"
            title="Click to enable audio for AI narration"
          >
            <Volume2 className="w-3.5 h-3.5 sm:w-4 sm:h-4" />
            <span className="text-[8px] sm:text-[10px] racing-text hidden sm:inline">
              UNLOCK AUDIO
            </span>
          </button>
        )}
        <SettingsDialog tuning={tuning} onTuningChange={onTuningChange} backendDefaults={backendDefaults} cameraSpecs={cameraSpecs} narrationConfig={narrationConfig} imageAnalysisEnabled={imageAnalysisEnabled} onImageAnalysisToggle={onImageAnalysisToggle} isIREnabled={isIREnabled} isSonarEnabled={isSonarEnabled} isMPU6050Enabled={isMPU6050Enabled} isRearSonarEnabled={isRearSonarEnabled} isCameraEnabled={isCameraEnabled} onIRToggle={onIRToggle} onSonarToggle={onSonarToggle} onRearSonarToggle={onRearSonarToggle} onMPU6050Toggle={onMPU6050Toggle} onCameraToggle={onCameraToggle} isAutopilotRunning={isAutopilotRunning} onResetDriver={onResetDriver} />
      </div>
    </header>
  );
};
