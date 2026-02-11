import { useState } from "react";
import { Video, VideoOff, Mic } from "lucide-react";

interface CameraFeedProps {
  isConnected: boolean;
  streamUrl?: string;
  isCameraEnabled?: boolean;
  onToggleCamera?: () => void;
  narrationEnabled?: boolean;
  narrationSpeaking?: boolean;
  narrationLastText?: string;
}

export const CameraFeed = ({ isConnected, streamUrl, isCameraEnabled = true, onToggleCamera, narrationEnabled = false, narrationSpeaking = false, narrationLastText = '' }: CameraFeedProps) => {
  const [hasError, setHasError] = useState(false);
  const [isLoaded, setIsLoaded] = useState(false);

  const showStream = isCameraEnabled && isConnected && streamUrl && !hasError;

  return (
    <div className="w-full h-full flex flex-col overflow-hidden">
      <div className="racing-text text-[6px] sm:text-[8px] text-muted-foreground text-center mb-px">LIVE FEED</div>
      
      <div className="flex-1 racing-panel overflow-hidden relative bg-card/80 min-h-0">
        {/* Camera Disabled - Show Off State */}
        {!isCameraEnabled ? (
          <div className="absolute inset-0 flex flex-col items-center justify-center gap-3">
            <VideoOff className="w-6 h-6 sm:w-8 sm:h-8 text-muted-foreground/50" />
            <div className="text-center">
              <div className="text-[7px] sm:text-[9px] text-muted-foreground racing-text mb-2">CAMERA DISABLED</div>
              <button
                onClick={onToggleCamera}
                className="px-3 py-1.5 text-[6px] sm:text-[7px] bg-primary/80 hover:bg-primary text-primary-foreground rounded border border-primary/50 transition-colors racing-text font-medium"
              >
                TURN ON CAMERA
              </button>
            </div>
          </div>
        ) : (
          <>
            {/* Live MJPEG stream */}
            {showStream && (
              <img
                src={streamUrl}
                alt="Live Camera Feed"
                className={`absolute inset-0 w-full h-full object-cover transition-opacity duration-300 ${isLoaded ? 'opacity-100' : 'opacity-0'}`}
                onLoad={() => setIsLoaded(true)}
                onError={() => setHasError(true)}
              />
            )}

            {/* Placeholder when no stream */}
            {!showStream && (
              <div className="absolute inset-0 flex flex-col items-center justify-center">
                {hasError ? (
                  <VideoOff className="w-3 h-3 sm:w-5 sm:h-5 text-destructive" />
                ) : (
                  <Video className={`w-3 h-3 sm:w-5 sm:h-5 ${isConnected ? 'text-primary animate-pulse' : 'text-muted-foreground'}`} />
                )}
                <span className="text-[5px] sm:text-[7px] text-muted-foreground racing-text mt-0.5">
                  {hasError ? 'FEED ERROR' : isConnected ? 'CONNECTING...' : 'NO SIGNAL'}
                </span>
              </div>
            )}
            
            {/* Scanline effect overlay */}
            <div className="absolute inset-0 pointer-events-none opacity-10">
              <div 
                className="w-full h-full"
                style={{
                  backgroundImage: 'repeating-linear-gradient(0deg, transparent, transparent 2px, hsl(var(--primary) / 0.1) 2px, hsl(var(--primary) / 0.1) 4px)',
                }}
              />
            </div>
            
            {/* Corner brackets */}
            <div className="absolute top-0.5 left-0.5 w-1.5 h-1.5 border-t border-l border-primary/50" />
            <div className="absolute top-0.5 right-0.5 w-1.5 h-1.5 border-t border-r border-primary/50" />
            <div className="absolute bottom-0.5 left-0.5 w-1.5 h-1.5 border-b border-l border-primary/50" />
            <div className="absolute bottom-0.5 right-0.5 w-1.5 h-1.5 border-b border-r border-primary/50" />
            
            {/* Recording indicator */}
            {showStream && isLoaded && (
              <div className="absolute top-0.5 right-2 flex items-center gap-px">
                <div className="w-1 h-1 rounded-full bg-destructive animate-pulse" />
                <span className="text-[4px] sm:text-[5px] text-destructive racing-text">REC</span>
              </div>
            )}

            {/* Narration indicator */}
            {narrationEnabled && showStream && isLoaded && (
              <div className="absolute bottom-1 left-1 flex items-center gap-0.5 bg-black/60 rounded px-1 py-0.5">
                <Mic className={`w-2 h-2 sm:w-2.5 sm:h-2.5 ${narrationSpeaking ? 'text-green-400 animate-pulse' : 'text-primary/70'}`} />
                <span className="text-[4px] sm:text-[5px] racing-text text-green-400/90 max-w-[80px] truncate">
                  {narrationSpeaking ? 'SPEAKING' : 'AI ON'}
                </span>
              </div>
            )}

            {/* Narration text overlay - shows when analysis is active */}
            {narrationEnabled && narrationLastText && showStream && isLoaded && (
              <div className="absolute bottom-4 left-1 right-1 bg-black/70 rounded px-1.5 py-1">
                <p className="text-[5px] sm:text-[7px] text-white/90 racing-text leading-relaxed">
                  🤖 {narrationLastText}
                </p>
              </div>
            )}
          </>
        )}
      </div>
    </div>
  );
};
