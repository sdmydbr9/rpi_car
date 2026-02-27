import { useMemo, useState } from "react";
import { SettingsDialog, TuningConstants, DEFAULT_TUNING } from "./SettingsDialog";
import { GamepadControlsDialog } from "./GamepadControlsDialog";
import type { CameraSpecs, NarrationConfig } from "../../lib/socketClient";
import { Loader2, RadioTower, Volume2, Wifi, Gamepad2 } from "lucide-react";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";

type NetworkMode = "wifi" | "hotspot";
type InputMode = "console" | "device";

interface HeaderProps {
  driverName?: string;
  position?: string;
  isConnected: boolean;
  networkMode?: NetworkMode;
  networkSwitching?: boolean;
  networkLinks?: {
    wifi: string;
    hotspot: string;
  };
  onNetworkModeSwitch?: (mode: NetworkMode) => void;
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
  // Gamepad status
  gamepadConnected?: boolean;
  inputMode?: InputMode | null;
}

export const Header = ({ 
  driverName = "HAMILTON", 
  position = "P1",
  isConnected,
  networkMode = "wifi",
  networkSwitching = false,
  networkLinks = {
    wifi: "http://raspberrypi.local:5000",
    hotspot: "http://192.168.4.1:5000",
  },
  onNetworkModeSwitch,
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
  gamepadConnected = false,
  inputMode = null,
}: HeaderProps) => {
  const [pendingNetworkMode, setPendingNetworkMode] = useState<NetworkMode | null>(null);
  const resolvedLinks = useMemo(() => ({
    wifi: networkLinks?.wifi || "http://raspberrypi.local:5000",
    hotspot: networkLinks?.hotspot || "http://192.168.4.1:5000",
  }), [networkLinks]);

  const targetRedirectLink = pendingNetworkMode ? resolvedLinks[pendingNetworkMode] : "";

  const openNetworkSwitchConfirm = (mode: NetworkMode) => {
    if (!onNetworkModeSwitch || networkSwitching || mode === networkMode) {
      return;
    }
    setPendingNetworkMode(mode);
  };

  const confirmNetworkSwitch = () => {
    if (!pendingNetworkMode || !onNetworkModeSwitch) {
      return;
    }
    onNetworkModeSwitch(pendingNetworkMode);
  };

  return (
    <>
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
        
        {/* Center: Connection Status + Network Mode Toggle + Gamepad Status */}
        <div className="flex items-center gap-1 sm:gap-2 text-[8px] sm:text-xs racing-text text-muted-foreground">
          <div className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-primary animate-pulse' : 'bg-destructive'}`} />
          <span className="hidden sm:inline">{isConnected ? 'CONNECTED' : 'OFFLINE'}</span>
          <div className="flex items-center rounded-full border border-primary/25 bg-background/40 p-px gap-0.5">
            <button
              type="button"
              disabled={networkSwitching || networkMode === "wifi" || !onNetworkModeSwitch}
              onClick={() => openNetworkSwitchConfirm("wifi")}
              className={`px-1.5 sm:px-2.5 py-0.5 rounded-full text-[7px] sm:text-[9px] font-bold transition-colors flex items-center gap-0.5 sm:gap-1 ${
                networkMode === "wifi"
                  ? "bg-primary/20 text-primary border border-primary/40"
                  : "text-muted-foreground hover:text-foreground hover:bg-muted/40"
              } ${networkSwitching ? "cursor-not-allowed opacity-70" : ""}`}
              title="Switch to WiFi mode"
            >
              <Wifi className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
              <span className="hidden sm:inline">WIFI</span>
            </button>
            <button
              type="button"
              disabled={networkSwitching || networkMode === "hotspot" || !onNetworkModeSwitch}
              onClick={() => openNetworkSwitchConfirm("hotspot")}
              className={`px-1.5 sm:px-2.5 py-0.5 rounded-full text-[7px] sm:text-[9px] font-bold transition-colors flex items-center gap-0.5 sm:gap-1 ${
                networkMode === "hotspot"
                  ? "bg-primary/20 text-primary border border-primary/40"
                  : "text-muted-foreground hover:text-foreground hover:bg-muted/40"
              } ${networkSwitching ? "cursor-not-allowed opacity-70" : ""}`}
              title="Switch to hotspot mode"
            >
              <RadioTower className="w-2.5 h-2.5 sm:w-3 sm:h-3" />
              <span className="hidden sm:inline">HOTSPOT</span>
            </button>
            {/* Gamepad/Device Mode Indicator */}
            {inputMode === "console" && (
              <GamepadControlsDialog gamepadConnected={gamepadConnected}>
                <button
                  type="button"
                  className="flex items-center gap-0.5 sm:gap-1 px-1.5 sm:px-2.5 py-0.5 rounded-full border border-primary/40 text-muted-foreground hover:text-foreground cursor-pointer transition-colors"
                  title="View gamepad controls"
                >
                  <Gamepad2 className={`w-2.5 h-2.5 sm:w-3 sm:h-3 ${gamepadConnected ? 'text-green-400' : 'text-destructive'}`} />
                  <span className="hidden sm:inline text-[7px] sm:text-[9px]">{gamepadConnected ? 'GAMEPAD' : 'NO GAMEPAD'}</span>
                </button>
              </GamepadControlsDialog>
            )}
          </div>
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
      <AlertDialog open={pendingNetworkMode !== null} onOpenChange={(open) => !open && setPendingNetworkMode(null)}>
        <AlertDialogContent className="max-w-md border-primary/30 bg-card/95 backdrop-blur-sm">
          <AlertDialogHeader>
            <AlertDialogTitle className="racing-text text-base sm:text-lg tracking-wider">
              Switch To {pendingNetworkMode === "hotspot" ? "HOTSPOT" : "WIFI"}?
            </AlertDialogTitle>
            <AlertDialogDescription className="text-xs sm:text-sm leading-relaxed">
              This will interrupt your connection and redirect you to:
              <a
                href={targetRedirectLink}
                className="mt-1.5 block text-primary underline underline-offset-4 break-all"
              >
                {targetRedirectLink}
              </a>
              <span className="mt-2 block">
                {pendingNetworkMode === "hotspot"
                  ? "WiFi will turn OFF and hotspot will turn ON."
                  : "Hotspot will turn OFF and WiFi will turn ON."}
              </span>
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogCancel className="border-border/60 bg-background/40 hover:bg-muted/50">
              Cancel
            </AlertDialogCancel>
            <AlertDialogAction
              onClick={confirmNetworkSwitch}
              disabled={networkSwitching}
              className="bg-primary/90 hover:bg-primary text-primary-foreground"
            >
              {networkSwitching ? (
                <>
                  <Loader2 className="w-3.5 h-3.5 animate-spin" />
                  Switching...
                </>
              ) : (
                "Switch & Redirect"
              )}
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </>
  );
};
