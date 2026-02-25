import { useState, useEffect, useRef, useCallback } from "react";
import { Video, VideoOff, Mic, RefreshCw, Sparkles, Loader2 } from "lucide-react";

const MAX_AUTO_RETRIES = 5;
const RETRY_DELAY_MS = 2000;

interface CameraFeedProps {
  isConnected: boolean;
  streamUrl?: string;
  isCameraEnabled?: boolean;
  onToggleCamera?: () => void;
  narrationEnabled?: boolean;
  narrationSpeaking?: boolean;
  narrationLastText?: string;
  onAnalyzeNow?: () => void;
  analyzeNowPending?: boolean;
}

export const CameraFeed = ({ isConnected, streamUrl, isCameraEnabled = true, onToggleCamera, narrationEnabled = false, narrationSpeaking = false, narrationLastText = '', onAnalyzeNow, analyzeNowPending = false }: CameraFeedProps) => {
  const [hasError, setHasError] = useState(false);
  const [isLoaded, setIsLoaded] = useState(false);
  const [retryCount, setRetryCount] = useState(0);
  const [imgKey, setImgKey] = useState(0); // Force remount of <img> on retry
  const [transport, setTransport] = useState<'h264' | 'mjpeg'>('h264');
  const retryTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const prevStreamUrlRef = useRef(streamUrl);

  // Reset state when streamUrl changes (reconnect to different server)
  useEffect(() => {
    if (streamUrl !== prevStreamUrlRef.current) {
      prevStreamUrlRef.current = streamUrl;
      setHasError(false);
      setIsLoaded(false);
      setRetryCount(0);
      setTransport('h264');
      setImgKey(k => k + 1);
      if (retryTimerRef.current) {
        clearTimeout(retryTimerRef.current);
        retryTimerRef.current = null;
      }
    }
  }, [streamUrl]);

  // Reset retry count when camera is toggled back on
  useEffect(() => {
    if (isCameraEnabled) {
      setRetryCount(0);
      setHasError(false);
      setIsLoaded(false);
      setTransport('h264');
      setImgKey(k => k + 1);
    }
  }, [isCameraEnabled]);

  // Clean up timer on unmount
  useEffect(() => {
    return () => {
      if (retryTimerRef.current) clearTimeout(retryTimerRef.current);
    };
  }, []);

  const handleError = useCallback(() => {
    setHasError(true);
    setIsLoaded(false);
    // Auto-retry up to MAX_AUTO_RETRIES times
    setRetryCount(prev => {
      const next = prev + 1;
      if (next <= MAX_AUTO_RETRIES) {
        retryTimerRef.current = setTimeout(() => {
          setHasError(false);
          setImgKey(k => k + 1);
        }, RETRY_DELAY_MS);
      }
      return next;
    });
  }, []);

  const handleManualRetry = useCallback(() => {
    setRetryCount(0);
    setHasError(false);
    setIsLoaded(false);
    setTransport('h264');
    setImgKey(k => k + 1);
  }, []);

  // Build actual MJPEG src with cache-busting per retry
  const effectiveMjpegUrl = streamUrl
    ? `${streamUrl}${streamUrl.includes('?') ? '&' : '?'}retry=${imgKey}`
    : undefined;
  const h264BaseUrl = streamUrl?.replace('/video_feed', '/video_feed_h264');
  const effectiveH264Url = h264BaseUrl
    ? `${h264BaseUrl}${h264BaseUrl.includes('?') ? '&' : '?'}retry=${imgKey}`
    : undefined;

  useEffect(() => {
    if (transport === 'h264' && !effectiveH264Url) {
      setTransport('mjpeg');
    }
  }, [transport, effectiveH264Url]);

  const showStream = isCameraEnabled && isConnected && !!(effectiveH264Url || effectiveMjpegUrl) && !hasError;
  const permanentError = hasError && retryCount > MAX_AUTO_RETRIES;

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
            {/* Preferred: hardware H264 feed, fallback: MJPEG */}
            {showStream && transport === 'h264' && effectiveH264Url && (
              <video
                key={`h264-${imgKey}`}
                src={effectiveH264Url}
                autoPlay
                muted
                playsInline
                className={`absolute inset-0 w-full h-full object-cover transition-opacity duration-300 ${isLoaded ? 'opacity-100' : 'opacity-0'}`}
                onLoadedData={() => { setIsLoaded(true); setRetryCount(0); }}
                onError={() => {
                  setIsLoaded(false);
                  setTransport('mjpeg');
                  setHasError(false);
                  setRetryCount(0);
                }}
              />
            )}

            {showStream && transport === 'mjpeg' && effectiveMjpegUrl && (
              <img
                key={imgKey}
                src={effectiveMjpegUrl}
                alt="Live Camera Feed"
                className={`absolute inset-0 w-full h-full object-cover transition-opacity duration-300 ${isLoaded ? 'opacity-100' : 'opacity-0'}`}
                onLoad={() => { setIsLoaded(true); setRetryCount(0); }}
                onError={handleError}
              />
            )}

            {/* Placeholder when no stream */}
            {!showStream && (
              <div className="absolute inset-0 flex flex-col items-center justify-center">
                {permanentError ? (
                  <>
                    <VideoOff className="w-3 h-3 sm:w-5 sm:h-5 text-destructive" />
                    <span className="text-[5px] sm:text-[7px] text-muted-foreground racing-text mt-0.5">FEED ERROR</span>
                    <button
                      onClick={handleManualRetry}
                      className="mt-1 flex items-center gap-0.5 px-2 py-0.5 text-[5px] sm:text-[6px] bg-primary/80 hover:bg-primary text-primary-foreground rounded border border-primary/50 transition-colors racing-text"
                    >
                      <RefreshCw className="w-2 h-2" />
                      RETRY
                    </button>
                  </>
                ) : hasError ? (
                  <>
                    <Video className="w-3 h-3 sm:w-5 sm:h-5 text-primary animate-pulse" />
                    <span className="text-[5px] sm:text-[7px] text-muted-foreground racing-text mt-0.5">
                      RECONNECTING ({retryCount}/{MAX_AUTO_RETRIES})
                    </span>
                  </>
                ) : (
                  <>
                    <Video className={`w-3 h-3 sm:w-5 sm:h-5 ${isConnected ? 'text-primary animate-pulse' : 'text-muted-foreground'}`} />
                    <span className="text-[5px] sm:text-[7px] text-muted-foreground racing-text mt-0.5">
                      {isConnected ? 'CONNECTING...' : 'NO SIGNAL'}
                    </span>
                  </>
                )}
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
