import { useCallback, useEffect, useRef, useState } from "react";
import { X, Crosshair, Play, Square, Target, Clock, Gauge, MapPin, Radar, RotateCcw } from "lucide-react";

interface HunterStatus {
  running: boolean;
  status: string;
  camera: string;
  mode: string;
  target: string;
  found: boolean;
  bbox: number[] | null;
  confidence: number;
  bbox_area: number;
  sensors: {
    front_laser: number;
    sonar: number;
    ir_left: boolean;
    ir_right: boolean;
    accel: number;
    gyro: number;
  };
  motors: { L: number; R: number };
  avoid: string;
  battery: number;
  sweep: boolean;
}

interface HunterPopupProps {
  isOpen: boolean;
  onClose: () => void;
  /** Base URL of the main Flask server, e.g. "http://192.168.29.210:5000" */
  serverBaseUrl: string;
  /** Direct MediaMTX WHEP URL for hunter stream, e.g. "http://192.168.29.210:8889/rover/whep" */
  whepUrl: string;
  /** Whether the hunter subprocess is active on the backend */
  hunterActive: boolean;
}

const POLL_MS = 300;

export const HunterPopup = ({ isOpen, onClose, serverBaseUrl, whepUrl, hunterActive }: HunterPopupProps) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const touchRef = useRef<HTMLDivElement>(null);
  const pcRef = useRef<RTCPeerConnection | null>(null);
  const pollRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const reconnRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const [status, setStatus] = useState<HunterStatus | null>(null);
  const [streamConnected, setStreamConnected] = useState(false);
  const [templateUrl, setTemplateUrl] = useState<string | null>(null);

  // All hunter API calls go through the Flask proxy on the main server
  const apiBase = `${serverBaseUrl}/api/hunter`;

  // ── WebRTC WHEP stream (direct to MediaMTX) ────────────
  const startStream = useCallback(async () => {
    if (!hunterActive) return;
    if (reconnRef.current) clearTimeout(reconnRef.current);
    if (pcRef.current) {
      try { pcRef.current.close(); } catch (_) { /* ignore */ }
    }
    setStreamConnected(false);

    const pc = new RTCPeerConnection({
      iceServers: [{ urls: "stun:stun.l.google.com:19302" }],
    });
    pcRef.current = pc;

    pc.addTransceiver("video", { direction: "recvonly" });
    pc.ontrack = (evt) => {
      if (videoRef.current) {
        videoRef.current.srcObject = evt.streams[0];
        setStreamConnected(true);
      }
    };
    pc.onconnectionstatechange = () => {
      if (pc.connectionState === "failed" || pc.connectionState === "disconnected") {
        setStreamConnected(false);
        reconnRef.current = setTimeout(startStream, 3000);
      }
    };

    try {
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
      // Wait for ICE gathering
      await new Promise<void>((resolve) => {
        if (pc.iceGatheringState === "complete") return resolve();
        const check = () => {
          if (pc.iceGatheringState === "complete") {
            pc.removeEventListener("icegatheringstatechange", check);
            resolve();
          }
        };
        pc.addEventListener("icegatheringstatechange", check);
        setTimeout(resolve, 4000);
      });

      const res = await fetch(whepUrl, {
        method: "POST",
        headers: { "Content-Type": "application/sdp" },
        body: pc.localDescription!.sdp,
      });
      if (!res.ok) throw new Error(`WHEP ${res.status}`);
      const answer = await res.text();
      await pc.setRemoteDescription({ type: "answer", sdp: answer });
    } catch (e) {
      console.error("WHEP error", e);
      reconnRef.current = setTimeout(startStream, 4000);
    }
  }, [whepUrl, hunterActive]);

  // ── Status polling ──────────────────────────────────────
  const pollStatus = useCallback(async () => {
    if (!hunterActive) return;
    try {
      const r = await fetch(`${apiBase}/status`);
      if (!r.ok) return;
      const d: HunterStatus = await r.json();
      setStatus(d);
      if (d.mode === "TOUCH_TRACKING") {
        setTemplateUrl(`${apiBase}/template.jpg?t=${Date.now()}`);
      } else {
        setTemplateUrl(null);
      }
    } catch (_) { /* ignore */ }
  }, [apiBase, hunterActive]);

  // ── Lifecycle ───────────────────────────────────────────
  useEffect(() => {
    if (!isOpen || !hunterActive) return;
    // Small delay to give the subprocess time to start its camera/stream
    const startDelay = setTimeout(() => {
      startStream();
      pollRef.current = setInterval(pollStatus, POLL_MS);
    }, 1500);
    return () => {
      clearTimeout(startDelay);
      if (pcRef.current) { try { pcRef.current.close(); } catch (_) { /* */ } pcRef.current = null; }
      if (pollRef.current) clearInterval(pollRef.current);
      if (reconnRef.current) clearTimeout(reconnRef.current);
      setStreamConnected(false);
      setStatus(null);
      setTemplateUrl(null);
    };
  }, [isOpen, hunterActive, startStream, pollStatus]);

  // ── Touch/click to pursue ──────────────────────────────
  const handleTouch = useCallback((e: React.MouseEvent | React.TouchEvent) => {
    e.preventDefault();
    if (!hunterActive) return;
    const vid = videoRef.current;
    if (!vid) return;
    const vw = vid.videoWidth || 1280;
    const vh = vid.videoHeight || 720;
    const rect = vid.getBoundingClientRect();
    const scale = Math.min(rect.width / vw, rect.height / vh);
    const renderedW = vw * scale;
    const renderedH = vh * scale;
    const offX = rect.left + (rect.width - renderedW) / 2;
    const offY = rect.top + (rect.height - renderedH) / 2;

    let cx: number, cy: number;
    if ("touches" in e && e.touches.length > 0) {
      cx = e.touches[0].clientX;
      cy = e.touches[0].clientY;
    } else if ("clientX" in e) {
      cx = (e as React.MouseEvent).clientX;
      cy = (e as React.MouseEvent).clientY;
    } else return;

    const nx = (cx - offX) / renderedW;
    const ny = (cy - offY) / renderedH;
    if (nx < 0 || nx > 1 || ny < 0 || ny > 1) return;

    fetch(`${apiBase}/touch`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: nx, y: ny }),
    }).then((r) => r.json()).then((d) => {
      if (d.ok) setTemplateUrl(`${apiBase}/template.jpg?t=${Date.now()}`);
    }).catch(() => {});
  }, [apiBase, hunterActive]);

  // ── Commands ────────────────────────────────────────────
  const sendCommand = useCallback((cmd: string) => {
    fetch(`${apiBase}/command`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cmd }),
    }).catch(() => {});
    if (cmd === "reset") setTemplateUrl(null);
  }, [apiBase]);

  if (!isOpen) return null;

  const s = status?.sensors;
  const m = status?.motors;
  const conf = status?.confidence ?? 0;
  const confPct = Math.round(conf * 100);
  const confTextColor = confPct > 60 ? "text-green-500" : confPct > 35 ? "text-yellow-400" : "text-destructive";
  const isActivelyTracking = status?.mode === "TOUCH_TRACKING" || status?.mode === "TRACKING";
  const hasTarget = status?.found ?? false;
  const reticlePosition = (() => {
    if (!status?.bbox || status.bbox.length !== 4) return null;
    const [b0, b1, b2, b3] = status.bbox;

    // Backend uses [x1, y1, x2, y2]; keep a fallback for legacy [x, y, w, h].
    const isXyxy = b2 > b0 && b3 > b1;
    const x1 = b0;
    const y1 = b1;
    const x2 = isXyxy ? b2 : b0 + b2;
    const y2 = isXyxy ? b3 : b1 + b3;
    const cx = (x1 + x2) / 2;
    const cy = (y1 + y2) / 2;

    const vid = videoRef.current;
    const vw = vid?.videoWidth || 1280;
    const vh = vid?.videoHeight || 720;
    if (!vid) {
      return {
        left: `${(cx / vw) * 100}%`,
        top: `${(cy / vh) * 100}%`,
      };
    }

    const rect = vid.getBoundingClientRect();
    const scale = Math.min(rect.width / vw, rect.height / vh);
    const renderedW = vw * scale;
    const renderedH = vh * scale;
    const padX = (rect.width - renderedW) / 2;
    const padY = (rect.height - renderedH) / 2;

    return {
      left: `${padX + (cx / vw) * renderedW}px`,
      top: `${padY + (cy / vh) * renderedH}px`,
    };
  })();

  return (
    <div className="fixed inset-0 z-[200] flex items-center justify-center bg-black/85 backdrop-blur-sm">
      <div className="w-[92vw] max-w-2xl h-[85vh] max-h-[600px] rounded-xl border border-primary/30 bg-card/95 flex flex-col overflow-hidden shadow-2xl shadow-primary/10">

        {/* Header Bar */}
        <div className="flex items-center justify-between px-3 py-2 border-b border-border/40 bg-card">
          <div className="flex items-center gap-2">
            <Target className="w-4 h-4 text-destructive" />
            <span className="racing-text text-xs sm:text-sm text-foreground tracking-wider">PURSUIT MODE</span>
            {!hunterActive ? (
              <span className="flex items-center gap-1 ml-2">
                <div className="w-1.5 h-1.5 rounded-full bg-yellow-400 animate-pulse" />
                <span className="racing-text text-[9px] text-yellow-400">INITIALIZING</span>
              </span>
            ) : isActivelyTracking ? (
              <span className="flex items-center gap-1 ml-2">
                <div className="w-1.5 h-1.5 rounded-full bg-destructive animate-pulse" />
                <span className="racing-text text-[9px] text-destructive">TRACKING</span>
              </span>
            ) : (
              <span className="racing-text text-[9px] text-muted-foreground ml-2">
                {status?.status || "IDLE"}
              </span>
            )}
          </div>

          {/* Telemetry Stats */}
          <div className="flex items-center gap-3">
            <div className="flex items-center gap-1">
              <Clock className="w-3 h-3 text-muted-foreground" />
              <span className="racing-text text-[9px] sm:text-[10px] text-muted-foreground">
                CAM {status?.camera || "--"}
              </span>
            </div>
            <div className="flex items-center gap-1">
              <Gauge className="w-3 h-3 text-muted-foreground" />
              <span className="racing-text text-[9px] sm:text-[10px] text-muted-foreground">
                {m ? `L:${m.L.toFixed(0)} R:${m.R.toFixed(0)}` : "-- / --"}
              </span>
            </div>
            <div className="flex items-center gap-1">
              <MapPin className="w-3 h-3 text-muted-foreground" />
              <span className={`racing-text text-[9px] sm:text-[10px] ${(status?.battery ?? 0) > 10 ? 'text-muted-foreground' : 'text-destructive'}`}>
                {(status?.battery ?? 0).toFixed(1)}V
              </span>
            </div>
          </div>

          <button
            onClick={onClose}
            className="w-7 h-7 rounded-md border border-border/50 flex items-center justify-center text-muted-foreground hover:text-foreground hover:border-foreground/40 transition-all"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* Viewfinder */}
        <div className="flex-1 relative overflow-hidden m-2 rounded-lg border border-border/30">
          <div className="absolute inset-0 bg-black">
            {hunterActive ? (
              <>
                <video
                  ref={videoRef}
                  autoPlay
                  playsInline
                  muted
                  className="w-full h-full"
                  style={{ objectFit: "contain", background: "#000" }}
                />
                <div
                  ref={touchRef}
                  className="absolute inset-0 cursor-crosshair"
                  style={{ zIndex: 5 }}
                  onClick={handleTouch}
                  onTouchStart={handleTouch}
                />
                {!streamConnected && (
                  <div className="absolute inset-0 flex flex-col items-center justify-center pointer-events-none" style={{ zIndex: 2 }}>
                    <Crosshair className="w-8 h-8 text-primary/40 mb-2 animate-pulse" />
                    <span className="racing-text text-[10px] sm:text-xs text-primary/50">CONNECTING TO ROVER STREAM</span>
                    <span className="racing-text text-[8px] text-muted-foreground mt-1">ESTABLISHING WHEP LINK...</span>
                  </div>
                )}
              </>
            ) : (
              <>
                {/* Simulated camera feed placeholder */}
                <div
                  className="absolute inset-0"
                  style={{
                    background: "radial-gradient(ellipse at center, hsl(var(--card)) 0%, hsl(0 0% 5%) 100%)",
                  }}
                />
                <div className="absolute inset-0 flex flex-col items-center justify-center pointer-events-none">
                  <div className="w-10 h-10 border-2 border-t-primary border-r-transparent border-b-transparent border-l-transparent rounded-full animate-spin mb-3" />
                  <span className="racing-text text-[10px] sm:text-xs text-primary/50">STARTING HUNTER SUBSYSTEM</span>
                  <span className="racing-text text-[8px] text-muted-foreground mt-1">RELEASING CAMERA &amp; INITIALIZING VISION</span>
                </div>
              </>
            )}

            {/* Grid overlay */}
            <svg className="absolute inset-0 w-full h-full opacity-15 pointer-events-none" style={{ zIndex: 6 }}>
              <defs>
                <pattern id="pursuit-grid" width="50" height="50" patternUnits="userSpaceOnUse">
                  <path d="M 50 0 L 0 0 0 50" fill="none" stroke="hsl(var(--primary))" strokeWidth="0.5" />
                </pattern>
              </defs>
              <rect width="100%" height="100%" fill="url(#pursuit-grid)" />
            </svg>

            {/* Center crosshair (faint, always present) */}
            <div className="absolute inset-0 flex items-center justify-center pointer-events-none opacity-20" style={{ zIndex: 6 }}>
              <Crosshair className="w-12 h-12 text-primary" />
            </div>

            {/* Scanlines */}
            <div
              className="absolute inset-0 pointer-events-none opacity-10"
              style={{
                zIndex: 6,
                backgroundImage: "repeating-linear-gradient(0deg, transparent, transparent 2px, hsl(var(--primary) / 0.15) 2px, hsl(var(--primary) / 0.15) 4px)",
              }}
            />

            {/* Corner brackets */}
            <div className="absolute top-2 left-2 w-4 h-4 border-t-2 border-l-2 border-primary/50 pointer-events-none" style={{ zIndex: 6 }} />
            <div className="absolute top-2 right-2 w-4 h-4 border-t-2 border-r-2 border-primary/50 pointer-events-none" style={{ zIndex: 6 }} />
            <div className="absolute bottom-2 left-2 w-4 h-4 border-b-2 border-l-2 border-primary/50 pointer-events-none" style={{ zIndex: 6 }} />
            <div className="absolute bottom-2 right-2 w-4 h-4 border-b-2 border-r-2 border-primary/50 pointer-events-none" style={{ zIndex: 6 }} />

            {/* "Tap to lock" hint when no target is active */}
            {hunterActive && streamConnected && !isActivelyTracking && !hasTarget && (
              <div className="absolute inset-0 flex flex-col items-center justify-center pointer-events-none" style={{ zIndex: 7 }}>
                <Crosshair className="w-8 h-8 text-primary/40 mb-2" />
                <span className="racing-text text-[10px] sm:text-xs text-primary/50">TAP TO LOCK TARGET</span>
                <span className="racing-text text-[8px] text-muted-foreground mt-1">CLICK ANYWHERE ON THE FEED</span>
              </div>
            )}

            {/* Target bullseye reticle when tracking */}
            {hasTarget && reticlePosition && (
              <div
                className="absolute pointer-events-none"
                style={{
                  left: reticlePosition.left,
                  top: reticlePosition.top,
                  transform: "translate(-50%, -50%)",
                  zIndex: 8,
                }}
              >
                {/* Outer ring */}
                <div className={`w-16 h-16 rounded-full border-2 ${isActivelyTracking ? 'border-destructive' : 'border-primary'} flex items-center justify-center ${isActivelyTracking ? 'animate-pulse' : ''}`}>
                  {/* Inner ring */}
                  <div className={`w-9 h-9 rounded-full border ${isActivelyTracking ? 'border-destructive/70' : 'border-primary/70'}`} />
                  {/* Center dot */}
                  <div className={`absolute w-2.5 h-2.5 rounded-full ${isActivelyTracking ? 'bg-destructive' : 'bg-primary'}`} />
                </div>
                {/* Crosshair tick marks */}
                <div className={`absolute top-1/2 -left-4 w-3 h-px ${isActivelyTracking ? 'bg-destructive' : 'bg-primary'}`} />
                <div className={`absolute top-1/2 -right-4 w-3 h-px ${isActivelyTracking ? 'bg-destructive' : 'bg-primary'}`} />
                <div className={`absolute -top-4 left-1/2 h-3 w-px ${isActivelyTracking ? 'bg-destructive' : 'bg-primary'}`} />
                <div className={`absolute -bottom-4 left-1/2 h-3 w-px ${isActivelyTracking ? 'bg-destructive' : 'bg-primary'}`} />
                {/* Label with percentage below reticle */}
                <div className="absolute -bottom-7 left-1/2 -translate-x-1/2 flex items-center gap-1.5 whitespace-nowrap">
                  <span className={`racing-text text-[9px] font-bold ${isActivelyTracking ? 'text-destructive' : 'text-primary'}`}>
                    {isActivelyTracking ? "LOCKED" : "TARGET"}
                  </span>
                  {isActivelyTracking && (
                    <span className={`racing-text text-[9px] font-bold ${confTextColor}`}>
                      {confPct}%
                    </span>
                  )}
                </div>
              </div>
            )}

            {/* Template preview thumbnail */}
            {templateUrl && (
              <div className="absolute top-2 right-8 w-14 h-14 rounded-md overflow-hidden border-2 border-primary/60" style={{ zIndex: 12, background: "#000" }}>
                <img src={templateUrl} alt="target" className="w-full h-full object-cover" />
              </div>
            )}
          </div>
        </div>

        {/* Footer Controls */}
        <div className="px-3 py-2 border-t border-border/40 bg-card">
          {/* Sensor readout row */}
          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-1">
              <div className={`w-2 h-2 rounded-full ${isActivelyTracking ? 'bg-destructive animate-pulse' : hasTarget ? 'bg-primary' : 'bg-muted-foreground/30'}`} />
              <span className="racing-text text-[9px] sm:text-[10px] text-muted-foreground">
                {isActivelyTracking ? "PURSUIT ACTIVE" : hasTarget ? "TARGET ACQUIRED" : "AWAITING TARGET"}
              </span>
            </div>
            <span className="racing-text text-[9px] text-muted-foreground">
              F:{s?.front_laser ?? "--"} S:{s?.sonar ?? "--"}{" "}
              IR:{s?.ir_left ? "L" : "."}{s?.ir_right ? "R" : "."}{" "}
              G:{(s?.accel ?? 0).toFixed(1)} | {status?.avoid ?? "--"}
            </span>
          </div>

          {/* Control buttons */}
          <div className="flex items-center justify-center gap-2">
            <button
              onClick={() => sendCommand("start")}
              disabled={!hunterActive}
              className={`
                h-9 px-4 rounded-lg border flex items-center justify-center gap-1.5
                text-[9px] sm:text-[11px] font-bold racing-text transition-all touch-feedback
                ${!hunterActive
                  ? 'opacity-40 pointer-events-none bg-card/80 border-border text-muted-foreground'
                  : 'bg-primary/20 border-primary text-primary hover:bg-primary/30 glow-teal'
                }
              `}
            >
              <Play className="w-3.5 h-3.5" />
              PURSUE
            </button>

            <button
              onClick={() => sendCommand("scan")}
              disabled={!hunterActive}
              className={`
                h-9 px-4 rounded-lg border flex items-center justify-center gap-1.5
                text-[9px] sm:text-[11px] font-bold racing-text transition-all touch-feedback
                ${!hunterActive
                  ? 'opacity-40 pointer-events-none bg-card/80 border-border text-muted-foreground'
                  : 'bg-primary/20 border-primary text-primary hover:bg-primary/30 glow-teal'
                }
              `}
            >
              <Radar className="w-3.5 h-3.5" />
              SCAN
            </button>

            <button
              onClick={() => sendCommand("stop")}
              disabled={!hunterActive}
              className={`
                h-9 px-4 rounded-lg border flex items-center justify-center gap-1.5
                text-[9px] sm:text-[11px] font-bold racing-text transition-all touch-feedback
                ${!hunterActive
                  ? 'opacity-40 pointer-events-none bg-card/80 border-border text-muted-foreground'
                  : 'bg-destructive/20 border-destructive text-destructive hover:bg-destructive/30'
                }
              `}
            >
              <Square className="w-3.5 h-3.5" />
              ABORT
            </button>

            <button
              onClick={() => sendCommand("reset")}
              disabled={!hunterActive}
              className={`
                h-9 px-4 rounded-lg border flex items-center justify-center gap-1.5
                text-[9px] sm:text-[11px] font-bold racing-text transition-all touch-feedback
                ${!hunterActive
                  ? 'opacity-40 pointer-events-none bg-card/80 border-border text-muted-foreground'
                  : 'bg-destructive/20 border-destructive text-destructive hover:bg-destructive/30'
                }
              `}
            >
              <RotateCcw className="w-3.5 h-3.5" />
              RESET
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
