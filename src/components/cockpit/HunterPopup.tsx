import { useCallback, useEffect, useRef, useState } from "react";

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
    front_sonar: number;
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
  const confColor = confPct > 60 ? "#00ff88" : confPct > 35 ? "#ffcc00" : "#ff3344";
  const statusColor = status?.found ? "#ffcc00" : status?.mode === "TOUCH_TRACKING" && !status?.found ? "#ff3344" : "#00ff88";

  return (
    <div
      className="fixed inset-0 z-[200] flex items-center justify-center"
      style={{ background: "rgba(0,0,0,0.85)" }}
      onClick={(e) => { if (e.target === e.currentTarget) onClose(); }}
    >
      <div
        className="relative w-[92vw] max-w-[900px] h-[80vh] max-h-[600px] rounded-lg overflow-hidden flex flex-col"
        style={{
          background: "#0a0a0a",
          border: "1px solid rgba(20, 184, 166, 0.4)",
          fontFamily: "'Courier New', monospace",
        }}
      >
        {/* ── Header ── */}
        <div
          className="flex items-center justify-between px-3 py-1.5 flex-shrink-0"
          style={{ background: "rgba(0,0,0,0.7)", borderBottom: "1px solid rgba(20,184,166,0.2)" }}
        >
          <div className="flex items-center gap-3">
            <span className="font-bold text-xs tracking-wider" style={{ color: "#00ff88" }}>
              HUNTER MODE
            </span>
            {!hunterActive ? (
              <span className="text-[10px] animate-pulse" style={{ color: "#ffcc00" }}>
                INITIALIZING...
              </span>
            ) : (
              <span className="text-[10px]" style={{ color: statusColor }}>
                {status?.status || "IDLE"}
              </span>
            )}
          </div>
          <div className="flex items-center gap-3">
            <span className="text-[10px]" style={{ color: "#888" }}>
              CAM {status?.camera || "--"}
            </span>
            <span className="text-[10px]" style={{ color: (status?.battery ?? 0) > 10 ? "#aaa" : "#ff3344" }}>
              {(status?.battery ?? 0).toFixed(1)}V
            </span>
            <button
              onClick={onClose}
              className="w-6 h-6 rounded-full flex items-center justify-center"
              style={{ background: "rgba(255,51,68,0.15)", border: "1px solid #ff3344", color: "#ff3344", fontSize: 12, cursor: "pointer" }}
            >
              ✕
            </button>
          </div>
        </div>

        {/* ── Video + touch overlay ── */}
        <div className="relative flex-1 min-h-0 flex items-center justify-center overflow-hidden" style={{ background: "#000" }}>
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
                className="absolute inset-0"
                style={{ cursor: "crosshair", zIndex: 5 }}
                onClick={handleTouch}
                onTouchStart={handleTouch}
              />
              {!streamConnected && (
                <div className="absolute inset-0 flex items-center justify-center" style={{ color: "#555", fontSize: 13, zIndex: 2 }}>
                  Connecting to rover stream...
                </div>
              )}
            </>
          ) : (
            <div className="flex flex-col items-center justify-center gap-3" style={{ color: "#555" }}>
              <div className="w-10 h-10 border-2 border-t-teal-400 border-r-transparent border-b-transparent border-l-transparent rounded-full animate-spin" />
              <span className="text-sm">Starting hunter subsystem...</span>
              <span className="text-[10px]" style={{ color: "#444" }}>Releasing camera &amp; initializing vision</span>
            </div>
          )}
          {/* Template preview thumbnail */}
          {templateUrl && (
            <div
              className="absolute top-2 right-2 w-16 h-16 rounded overflow-hidden"
              style={{ border: "2px solid #00ff88", zIndex: 12, background: "#000" }}
            >
              <img src={templateUrl} alt="target" className="w-full h-full object-cover" />
            </div>
          )}
        </div>

        {/* ── Bottom HUD ── */}
        <div className="flex-shrink-0 px-3 py-1.5" style={{ background: "rgba(0,0,0,0.7)", borderTop: "1px solid rgba(20,184,166,0.2)" }}>
          {/* Sensor readout */}
          <div className="text-[10px] mb-1" style={{ color: "#888" }}>
            <span>
              F:{s?.front_laser ?? "--"} R:{s?.front_sonar ?? "--"}{" "}
              IR:{s?.ir_left ? "L" : "."}{s?.ir_right ? "R" : "."}{" "}
              G:{(s?.accel ?? 0).toFixed(1)}
            </span>
            {(status?.mode === "TOUCH_TRACKING" || status?.mode === "TRACKING") && (
              <span className="ml-2">
                LOCK
                <span
                  className="inline-block ml-1 rounded-sm overflow-hidden align-middle"
                  style={{ width: 50, height: 5, background: "#333" }}
                >
                  <span
                    className="block h-full transition-all"
                    style={{ width: `${confPct}%`, background: confColor }}
                  />
                </span>
              </span>
            )}
          </div>
          <div className="text-[10px] mb-1.5" style={{ color: "#888" }}>
            M: L:{(m?.L ?? 0).toFixed(0)} R:{(m?.R ?? 0).toFixed(0)} | {status?.avoid ?? "--"}
          </div>

          {/* Control buttons */}
          <div className="flex gap-2 justify-center flex-wrap">
            <button
              onClick={() => sendCommand("start")}
              disabled={!hunterActive}
              className="px-4 py-1.5 rounded text-xs font-bold tracking-wider transition-colors disabled:opacity-40"
              style={{
                border: "1px solid #00ff88",
                background: "rgba(0,255,136,0.08)",
                color: "#00ff88",
                cursor: hunterActive ? "pointer" : "not-allowed",
                fontFamily: "'Courier New', monospace",
              }}
            >
              START
            </button>
            <button
              onClick={() => sendCommand("stop")}
              disabled={!hunterActive}
              className="px-4 py-1.5 rounded text-xs font-bold tracking-wider transition-colors disabled:opacity-40"
              style={{
                border: "1px solid #ff3344",
                background: "rgba(255,51,68,0.08)",
                color: "#ff3344",
                cursor: hunterActive ? "pointer" : "not-allowed",
                fontFamily: "'Courier New', monospace",
              }}
            >
              STOP
            </button>
            <button
              onClick={() => sendCommand("scan")}
              disabled={!hunterActive}
              className="px-4 py-1.5 rounded text-xs font-bold tracking-wider transition-colors disabled:opacity-40"
              style={{
                border: "1px solid #00ff88",
                background: "rgba(0,255,136,0.08)",
                color: "#00ff88",
                cursor: hunterActive ? "pointer" : "not-allowed",
                fontFamily: "'Courier New', monospace",
              }}
            >
              SCAN
            </button>
            <button
              onClick={() => sendCommand("reset")}
              disabled={!hunterActive}
              className="px-4 py-1.5 rounded text-xs font-bold tracking-wider transition-colors disabled:opacity-40"
              style={{
                border: "1px solid #ff3344",
                background: "rgba(255,51,68,0.08)",
                color: "#ff3344",
                cursor: hunterActive ? "pointer" : "not-allowed",
                fontFamily: "'Courier New', monospace",
              }}
            >
              RESET
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
