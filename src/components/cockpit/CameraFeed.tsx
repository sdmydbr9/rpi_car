import { useState, useEffect, useRef, useMemo } from "react";
import { Video, VideoOff, Mic, Sparkles, Loader2 } from "lucide-react";
import { IrisOverlay, useIrisAnimation } from "@/components/ui/iris-overlay";

interface CameraFeedProps {
  isConnected: boolean;
  streamUrl?: string;
  isCameraEnabled?: boolean;
  cameraResolution?: string;
  onToggleCamera?: () => void;
  narrationEnabled?: boolean;
  narrationSpeaking?: boolean;
  narrationLastText?: string;
  onAnalyzeNow?: () => void;
  analyzeNowPending?: boolean;
}

const resolveCameraAspectRatio = (resolution?: string): number => {
  const normalized = (resolution || "").toLowerCase().trim();
  if (normalized === "low") return 4 / 3;
  if (normalized === "medium" || normalized === "high") return 16 / 9;

  const match = normalized.match(/(\d+)\s*x\s*(\d+)/);
  if (match) {
    const width = Number(match[1]);
    const height = Number(match[2]);
    if (width > 0 && height > 0) return width / height;
  }
  return 4 / 3;
};

export const CameraFeed = ({ isConnected, streamUrl, isCameraEnabled = true, cameraResolution, onToggleCamera, narrationEnabled = false, narrationSpeaking = false, narrationLastText = '', onAnalyzeNow, analyzeNowPending = false }: CameraFeedProps) => {
  const [isLoaded, setIsLoaded] = useState(false);
  const [viewerKey, setViewerKey] = useState(0);
  const [containerAspectRatio, setContainerAspectRatio] = useState(16 / 9);
  const prevStreamUrlRef = useRef(streamUrl);
  const viewportRef = useRef<HTMLDivElement>(null);

  /* ---- Iris animation ---- */
  const irisPhase = useIrisAnimation(isCameraEnabled);

  // Reset iframe when stream URL changes.
  useEffect(() => {
    if (streamUrl !== prevStreamUrlRef.current) {
      prevStreamUrlRef.current = streamUrl;
      setIsLoaded(false);
      setViewerKey(k => k + 1);
    }
  }, [streamUrl]);

  // Reload viewer when camera is toggled on.
  useEffect(() => {
    if (isCameraEnabled) {
      setIsLoaded(false);
      setViewerKey(k => k + 1);
    }
  }, [isCameraEnabled]);

  useEffect(() => {
    const container = viewportRef.current;
    if (!container) return;

    const updateAspectRatio = () => {
      if (container.clientWidth > 0 && container.clientHeight > 0) {
        setContainerAspectRatio(container.clientWidth / container.clientHeight);
      }
    };

    updateAspectRatio();

    if (typeof ResizeObserver === "undefined") {
      window.addEventListener("resize", updateAspectRatio);
      return () => {
        window.removeEventListener("resize", updateAspectRatio);
      };
    }

    const observer = new ResizeObserver(updateAspectRatio);
    observer.observe(container);

    return () => {
      observer.disconnect();
    };
  }, []);

  const effectiveViewerUrl = streamUrl
    ? `${streamUrl}${streamUrl.includes('?') ? '&' : '?'}autoplay=1&muted=1&controls=0&playsinline=1&disablepictureinpicture=1`
    : undefined;
  const showStream = isCameraEnabled && isConnected && !!effectiveViewerUrl;
  const streamAspectRatio = useMemo(() => resolveCameraAspectRatio(cameraResolution), [cameraResolution]);
  const iframeCoverStyle = useMemo(() => {
    const safeContainerAspectRatio = containerAspectRatio > 0 ? containerAspectRatio : streamAspectRatio;

    if (safeContainerAspectRatio > streamAspectRatio) {
      return {
        width: "100%",
        height: `${(safeContainerAspectRatio / streamAspectRatio) * 100}%`,
        left: 0,
        top: "50%",
        transform: "translateY(-50%)",
      };
    }

    return {
      width: `${(streamAspectRatio / safeContainerAspectRatio) * 100}%`,
      height: "100%",
      left: "50%",
      top: 0,
      transform: "translateX(-50%)",
    };
  }, [containerAspectRatio, streamAspectRatio]);

  return (
    <div className="w-full h-full overflow-hidden">
      <div ref={viewportRef} className="w-full h-full overflow-hidden relative min-h-0 bg-black">
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
            {/* Iris lens animation overlay */}
            <IrisOverlay phase={irisPhase} />

            {showStream && effectiveViewerUrl && (
              <iframe
                key={viewerKey}
                src={effectiveViewerUrl}
                title="Live Camera Feed"
                className="absolute border-0 pointer-events-none"
                style={iframeCoverStyle}
                onLoad={() => setIsLoaded(true)}
                allow="autoplay; picture-in-picture"
                sandbox="allow-scripts allow-same-origin allow-forms allow-pointer-lock"
              />
            )}

            {/* Placeholder when no stream */}
            {!showStream && (
              <div className="absolute inset-0 flex flex-col items-center justify-center">
                <Video className={`w-3 h-3 sm:w-5 sm:h-5 ${isConnected ? 'text-primary animate-pulse' : 'text-muted-foreground'}`} />
                <span className="text-[5px] sm:text-[7px] text-muted-foreground racing-text mt-0.5">
                  {isConnected ? 'CONNECTING...' : 'NO SIGNAL'}
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

            {/* Analyze current frame button */}
            {isCameraEnabled && (
              <button
                type="button"
                onClick={(e) => {
                  e.stopPropagation();
                  onAnalyzeNow?.();
                }}
                disabled={!isConnected || !onAnalyzeNow || analyzeNowPending}
                title={analyzeNowPending ? 'Analyzing frame...' : 'Analyze current frame'}
                className="absolute top-0.5 left-2 z-10 flex items-center justify-center w-4 h-4 sm:w-5 sm:h-5 rounded border border-primary/50 bg-black/60 text-primary hover:bg-primary/20 disabled:opacity-40 disabled:cursor-not-allowed transition-colors"
              >
                {analyzeNowPending ? (
                  <Loader2 className="w-2 h-2 sm:w-2.5 sm:h-2.5 animate-spin" />
                ) : (
                  <Sparkles className="w-2 h-2 sm:w-2.5 sm:h-2.5" />
                )}
              </button>
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
