import { useCallback, useContext, createContext, useEffect, useMemo, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";
import { buildMockDebugData, DebugSample, rowsToSamples } from "@/lib/mockDebugData";
import { Button } from "@/components/ui/button";
import { useToast } from "@/hooks/use-toast";
import {
  ArrowLeft, Download, FileText, Clock, Activity, Pause, Play, Search, X, RefreshCw, Camera,
} from "lucide-react";
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
  BarChart, Bar, ScatterChart, Scatter, ZAxis,
  Cell, ReferenceLine,
} from "recharts";

// ─── colour palette ──────────────────────────────────────────────
const C = {
  teal:     "hsl(177,100%,40%)",
  red:      "hsl(345,100%,55%)",
  orange:   "hsl(30,100%,55%)",
  green:    "hsl(142,76%,46%)",
  purple:   "hsl(260,80%,60%)",
  blue:     "hsl(200,80%,55%)",
  yellow:   "hsl(48,100%,55%)",
  pink:     "hsl(320,80%,60%)",
  grid:     "hsl(200,20%,12%)",
  axis:     "hsl(200,10%,35%)",
  axisText: "hsl(200,10%,50%)",
};

const tooltipStyle = {
  contentStyle: {
    backgroundColor: "hsl(200,30%,6%)",
    border: "1px solid hsl(177,100%,25%)",
    borderRadius: "3px",
    fontSize: "9px",
    fontFamily: "monospace",
    padding: "4px 6px",
  },
  labelStyle: { color: "hsl(180,5%,85%)", fontSize: "9px" },
};

const tickStyle = { fontSize: 8, fill: C.axisText };

// ─── time window filter ──────────────────────────────────────────
type TimeWindow = "all" | "10" | "30" | "60" | "300" | "custom";

const TIME_WINDOWS: { label: string; value: TimeWindow; seconds: number | null }[] = [
  { label: "ALL",   value: "all", seconds: null },
  { label: "10 s",  value: "10",  seconds: 10   },
  { label: "30 s",  value: "30",  seconds: 30   },
  { label: "1 min", value: "60",  seconds: 60   },
  { label: "5 min", value: "300", seconds: 300  },
];

/** Convert epoch-ms to a value suitable for <input type="datetime-local"> (local time) */
function epochToInputValue(ms: number): string {
  const d = new Date(ms);
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth()+1)}-${p(d.getDate())}T${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

/**
 * Parse a datetime-local input string as LOCAL time.
 * new Date("YYYY-MM-DDTHH:mm:ss") has browser-specific UTC vs local-time
 * ambiguity. Using the numeric Date constructor is always local time.
 */
function localInputToEpoch(val: string): number {
  if (!val) return NaN;
  const [datePart, timePart] = val.split("T");
  if (!datePart || !timePart) return NaN;
  const [y, m, d] = datePart.split("-").map(Number);
  const [h, min, s = 0] = timePart.split(":").map(Number);
  // new Date(y, m-1, d, h, min, s) is ALWAYS local time — no UTC ambiguity
  return new Date(y, m - 1, d, h, min, s).getTime();
}

function filterByWindow(data: DebugSample[], seconds: number | null): DebugSample[] {
  if (!seconds || data.length === 0) return data;
  const cutoff = Date.now() - seconds * 1000;
  const filtered = data.filter(r => Number(r.timestamp) >= cutoff);
  return filtered.length ? filtered : data.slice(-seconds);
}

function filterByCustomRange(data: DebugSample[], from: string, to: string): DebugSample[] {
  const fromMs = localInputToEpoch(from);
  const toMs   = localInputToEpoch(to);
  const lo = isNaN(fromMs) ? -Infinity : fromMs;
  const hi = isNaN(toMs)   ? Infinity  : toMs + 59_999; // +59 999 ms covers the rest of the selected second/minute
  return data.filter(r => Number(r.timestamp) >= lo && Number(r.timestamp) <= hi);
}

// ─── export helpers ───────────────────────────────────────────────
function exportCSV(data: DebugSample[]) {
  if (!data.length) return;
  const keys = Object.keys(data[0]) as (keyof DebugSample)[];
  const csv  = [keys.join(","), ...data.map(r => keys.map(k => r[k]).join(","))].join("\n");
  const a    = document.createElement("a");
  a.href     = URL.createObjectURL(new Blob([csv], { type: "text/csv" }));
  a.download = `debug_${Date.now()}.csv`;
  a.click();
}

function exportJSON(data: DebugSample[]) {
  const a    = document.createElement("a");
  a.href     = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: "application/json" }));
  a.download = `debug_${Date.now()}.json`;
  a.click();
}

// ─── context that forces all LazyCharts to render during PNG export ──
const ForceShowCtx = createContext(false);

// ─── panel wrapper ────────────────────────────────────────────────
const Panel = ({
  title, children, className = "",
}: { title: string; children: React.ReactNode; className?: string }) => (
  // contain: layout style paint — isolates each panel's layout/style recalc subtree
  <div className={`bg-card/50 border border-border/20 rounded overflow-hidden flex flex-col ${className}`} style={{ contain: "layout style paint" }}>
    <div className="px-2 py-1 border-b border-border/20 flex-shrink-0">
      <span className="text-[9px] font-bold uppercase tracking-widest text-muted-foreground">{title}</span>
    </div>
    <div className="flex-1 min-h-0 p-1">{children}</div>
  </div>
);

// ─── chart width measurement (single debounced ResizeObserver) ────
function useChartWidth(ref: React.RefObject<HTMLDivElement | null>): number {
  const [w, setW] = useState(0);
  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    // Read once after first paint (inside useEffect = no render-phase forced reflow)
    setW(Math.floor(el.getBoundingClientRect().width));
    let timer: ReturnType<typeof setTimeout>;
    const obs = new ResizeObserver(entries => {
      clearTimeout(timer);
      timer = setTimeout(() => setW(Math.floor(entries[0].contentRect.width)), 150);
    });
    obs.observe(el);
    return () => { obs.disconnect(); clearTimeout(timer); };
  }, [ref]);
  return w;
}

// ─── lazy-mount: skip rendering off-screen chart trees ────────────
function LazyChart({
  height, children, scrollRoot,
}: {
  height: number;
  children: React.ReactNode;
  scrollRoot?: React.RefObject<HTMLDivElement | null>;
}) {
  const elRef = useRef<HTMLDivElement>(null);
  const [show, setShow] = useState(false);
  useEffect(() => {
    const el = elRef.current;
    if (!el || show) return;
    const obs = new IntersectionObserver(
      ([e]) => { if (e.isIntersecting) { setShow(true); obs.disconnect(); } },
      { root: scrollRoot?.current ?? null, rootMargin: "400px" },
    );
    obs.observe(el);
    return () => obs.disconnect();
  }, [show, scrollRoot]);
  const forceShow = useContext(ForceShowCtx);
  return <div ref={elRef} style={{ minHeight: height }}>{(show || forceShow) && children}</div>;
}

// ─── mode flag colours ────────────────────────────────────────────
const MODE_COLORS: Record<string, string> = {
  IDLE:  "hsl(177,100%,35%)",
  SLOW:  "hsl(48,100%,50%)",
  CRAWL: "hsl(30,100%,50%)",
  STOP:  "hsl(345,100%,50%)",
  AVOID: "hsl(260,80%,55%)",
};

// ─── live-data hook ───────────────────────────────────────────────
const DEBUG_API = `http://${window.location.hostname}:5001/api/snapshot`;
const POLL_MS   = 1000;   // 1 s — halves request rate vs the old 500 ms
const MAX_LOCAL = 3600;   // rows kept in browser memory

function useDebugData(paused: boolean, _windowSeconds: number | null) {
  const [samples,  setSamples]  = useState<DebugSample[]>([]);
  const [rateHz,   setRateHz]   = useState<number>(0);
  const [elapsed,  setElapsed]  = useState<number>(0);
  const [status,   setStatus]   = useState<"connecting" | "live" | "error" | "mock">("connecting");
  const errorCount  = useRef(0);
  const seqTailRef  = useRef<number>(-1);   // last seq_tail received from server
  const localBufRef = useRef<DebugSample[]>([]);  // accumulated rows (client-side ring)

  const fetchData = useCallback(async () => {
    if (paused) return;
    try {
      const isFirst = seqTailRef.current < 0;
      const params = new URLSearchParams({ max_rows: String(MAX_LOCAL) });
      if (!isFirst) params.set("since_seq", String(seqTailRef.current));
      const res = await fetch(`${DEBUG_API}?${params}`, { signal: AbortSignal.timeout(2000) });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const json = await res.json() as {
        fields: string[]; rows: unknown[][]; rate_hz: number; session_elapsed_s: number; seq_tail: number;
      };
      if (json.seq_tail != null) seqTailRef.current = json.seq_tail;
      const incoming = rowsToSamples(json.fields, json.rows);
      if (incoming.length > 0 || isFirst) {
        localBufRef.current = isFirst
          ? incoming
          : [...localBufRef.current, ...incoming].slice(-MAX_LOCAL);
      }
      const live = localBufRef.current;
      setSamples(live.length ? live : buildMockDebugData(30));
      setRateHz(json.rate_hz ?? 0);
      setElapsed(json.session_elapsed_s ?? 0);
      setStatus("live");
      errorCount.current = 0;
    } catch {
      errorCount.current += 1;
      if (errorCount.current >= 3) {
        setStatus("error");
        if (localBufRef.current.length === 0) setSamples(buildMockDebugData(30));
      }
    }
  // windowSeconds filtering is done client-side via filterByWindow — no server dep needed
  }, [paused]);

  useEffect(() => {
    fetchData();
    const id = setInterval(fetchData, POLL_MS);
    return () => clearInterval(id);
  }, [fetchData]);

  // Switch to mock badge if we've never got real data and server is down
  useEffect(() => {
    if (status === "error" && localBufRef.current.length > 0 && localBufRef.current[0].timestamp > 0) {
      setStatus(errorCount.current >= 3 ? "mock" : "error");
    }
  }, [status]);

  return { samples, rateHz, elapsed, status };
}

// ─── Debug page ───────────────────────────────────────────────────
const Debug = () => {
  const navigate   = useNavigate();
  const { toast }  = useToast();
  const [timeWindow,   setTimeWindow]   = useState<TimeWindow>("all");
  const [isPaused,     setIsPaused]     = useState(false);
  const [searchQuery,  setSearchQuery]  = useState("");
  const [customFrom,   setCustomFrom]   = useState("");
  const [customTo,     setCustomTo]     = useState("");
  const [isExporting,  setIsExporting]  = useState(false);
  // Single scroll-container ref: used for ResizeObserver (width) + IO root (lazy charts)
  const chartAreaRef = useRef<HTMLDivElement>(null);
  const containerW   = useChartWidth(chartAreaRef);
  // full-width chart: subtract p-2 padding (16) + panel border (2) + panel inner p-1 (8) + fudge (2)
  const fullW = Math.max(0, containerW - 28);
  // half-width chart: split container minus gap-2 (8), then subtract same per-panel overhead
  const halfW = Math.max(0, Math.floor((containerW - 8) / 2) - 28);

  const windowSeconds = useMemo(
    () => TIME_WINDOWS.find(w => w.value === timeWindow)?.seconds ?? null,
    [timeWindow],
  );

  const { samples, rateHz, elapsed, status } = useDebugData(isPaused, windowSeconds);

  const data = useMemo(() => {
    const base = timeWindow === "custom"
      ? filterByCustomRange(samples, customFrom, customTo)
      : filterByWindow(samples, windowSeconds);
    if (!searchQuery) return base;
    const q = searchQuery.toLowerCase();
    return base.filter(r =>
      r.obstacle_state.toLowerCase().includes(q) ||
      new Date(r.timestamp).toLocaleString().toLowerCase().includes(q),
    );
  }, [samples, windowSeconds, timeWindow, customFrom, customTo, searchQuery]);

  const chartData = useMemo(() => data.map((r, i) => ({ ...r, t: i })), [data]);

  const scatterAccel = useMemo(() => data.map(r => ({
    x: r.accel_x, y: r.accel_y, z: Math.abs(r.accel_z),
  })), [data]);

  const scatterMag = useMemo(() => data.map((r, i) => ({
    x: r.mag_x, y: r.mag_y, t: i,
  })), [data]);

  // Pre-compute derived chart data to avoid inline array allocations inside JSX
  const driftData = useMemo(() => chartData.map(d => ({
    t: d.t, drift: +(d.rpm_rear_right - d.rpm_rear_left).toFixed(2),
  })), [chartData]);

  const pidData = useMemo(() => chartData.map(d => ({
    t: d.t, pid: d.pid_correction, err: d.heading_error_deg,
  })), [chartData]);

  // Downsample for the Mode Flags colour strips: no more than 300 divs per lane
  // (each strip is only a few px wide at 300 pts, so visually identical to 3600)
  const FLAG_SEGS = 300;
  const flagData = useMemo(() => {
    if (data.length <= FLAG_SEGS) return data;
    const step = Math.ceil(data.length / FLAG_SEGS);
    return data.filter((_, i) => i % step === 0);
  }, [data]);

  const elapsedStr = useMemo(() => {
    const s = Math.floor(elapsed);
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    const sec = s % 60;
    return h ? `${h}h ${m}m ${sec}s` : m ? `${m}m ${sec}s` : `${sec}s`;
  }, [elapsed]);

  const badgeClass = status === "live"
    ? "bg-primary/20 text-primary border-primary/30"
    : status === "error" || status === "connecting"
      ? "bg-yellow-500/20 text-yellow-400 border-yellow-500/30"
      : "bg-zinc-500/20 text-zinc-400 border-zinc-500/30";

  const badgeLabel = status === "live"     ? "LIVE"
                   : status === "mock"     ? "MOCK"
                   : status === "error"    ? "OFFLINE"
                   : "CONNECTING";

  // Auto-fill custom range from first/last sample epoch-ms timestamp when CUSTOM is selected
  const handleSelectCustom = () => {
    setTimeWindow("custom");
    if (samples.length > 0) {
      setCustomFrom(epochToInputValue(samples[0].timestamp));
      setCustomTo(epochToInputValue(samples[samples.length - 1].timestamp));
    }
  };

  // Export the full chart grid as PNG — runs in the background, shows toast notifications
  function handleExportPNG() {
    const el = chartAreaRef.current;
    if (!el || isExporting) return;

    // Immediately show feedback — export runs in background
    toast({
      title: "Exporting PNG…",
      description: "Capturing full dashboard. Download will start shortly.",
    });
    setIsExporting(true);

    void (async () => {
      const parent = el.parentElement as HTMLElement;
      const saveEl  = { flex: el.style.flex, overflow: el.style.overflow, height: el.style.height };
      const savePar = { overflow: parent.style.overflow, height: parent.style.height };
      try {
        // Wait for React to commit forceShow=true so all LazyCharts render their SVG trees
        await new Promise(r => setTimeout(r, 600));

        // Break el out of flex constraints so it stretches to full scroll height
        el.style.flex     = "0 0 auto";
        el.style.overflow = "visible";
        el.style.height   = el.scrollHeight + "px";
        parent.style.overflow = "visible";
        parent.style.height   = "auto";

        // Two rAF passes: flex layout recompute + paint settle
        await new Promise<void>(res => requestAnimationFrame(() => requestAnimationFrame(() => res())));

        const { default: html2canvas } = await import("html2canvas");
        const canvas = await html2canvas(el, {
          logging:      false,
          useCORS:      true,
          scale:        1.5,
          scrollY:      0,
          windowWidth:  el.offsetWidth,
          windowHeight: el.scrollHeight,
          onclone: (clonedDoc) => {
            // html2canvas renders into an off-screen iframe and doesn't inherit CSS custom
            // properties from external stylesheets. Copy all --vars from live :root as
            // inline styles on the cloned root so dark-theme colors are preserved.
            const cs = getComputedStyle(document.documentElement);
            const clonedRoot = clonedDoc.documentElement;
            for (const sheet of Array.from(document.styleSheets)) {
              try {
                for (const rule of Array.from(sheet.cssRules)) {
                  if (rule instanceof CSSStyleRule && /^:root/.test(rule.selectorText)) {
                    for (let i = 0; i < rule.style.length; i++) {
                      const prop = rule.style.item(i);
                      if (prop.startsWith("--")) {
                        clonedRoot.style.setProperty(prop, cs.getPropertyValue(prop).trim());
                      }
                    }
                  }
                }
              } catch { /* skip cross-origin sheets */ }
            }
          },
        });

        const rangeLabel = timeWindow === "custom"
          ? `${customFrom || "start"}_to_${customTo || "end"}`.replace(/[T:\s]/g, "-")
          : timeWindow;
        const fname = `debug_${rangeLabel}_${Date.now()}.png`;
        const a = document.createElement("a");
        a.href = canvas.toDataURL("image/png");
        a.download = fname;
        a.click();

        toast({ title: "PNG downloaded", description: fname });
      } catch (err) {
        console.error("PNG export failed:", err);
        toast({ title: "Export failed", description: String(err), variant: "destructive" });
      } finally {
        el.style.flex     = saveEl.flex;
        el.style.overflow = saveEl.overflow;
        el.style.height   = saveEl.height;
        parent.style.overflow = savePar.overflow;
        parent.style.height   = savePar.height;
        setIsExporting(false);
      }
    })();
  }

  return (
    <div className="h-[100dvh] w-full flex flex-col bg-background overflow-hidden font-rajdhani">

      {/* ── Header ─────────────────────────────────────────────── */}
      <div className="h-9 flex items-center justify-between px-3 border-b border-border/40 bg-card/80 flex-shrink-0">
        <div className="flex items-center gap-2">
          <Button variant="ghost" size="icon" className="h-6 w-6" onClick={() => navigate("/")}>
            <ArrowLeft className="h-3.5 w-3.5" />
          </Button>
          <Activity className="h-3.5 w-3.5 text-primary" />
          <span className="text-xs font-semibold tracking-wider text-foreground uppercase">Debug Log</span>
          <span className="text-[9px] text-muted-foreground font-mono">
            [{data.length} rows | {rateHz.toFixed(0)} Hz | session {elapsedStr}]
          </span>
          <div className={`px-1.5 py-0.5 rounded text-[8px] font-bold uppercase tracking-widest border ${badgeClass} ${
            status === "live" && !isPaused ? "animate-pulse" : ""
          }`}>
            {isPaused ? "PAUSED" : badgeLabel}
          </div>
        </div>

        <div className="flex items-center gap-1">
          <div className="relative">
            <Search className="absolute left-1.5 top-1/2 -translate-y-1/2 h-2.5 w-2.5 text-muted-foreground" />
            <input
              value={searchQuery}
              onChange={e => setSearchQuery(e.target.value)}
              placeholder="Filter state…"
              className="h-6 w-24 pl-5 pr-5 text-[10px] bg-secondary/60 border border-border/40 rounded text-foreground placeholder:text-muted-foreground focus:outline-none focus:border-primary/60 font-mono"
            />
            {searchQuery && (
              <button onClick={() => setSearchQuery("")} className="absolute right-1 top-1/2 -translate-y-1/2">
                <X className="h-2.5 w-2.5 text-muted-foreground" />
              </button>
            )}
          </div>
          <Button variant="ghost" size="icon" className="h-6 w-6" onClick={() => setIsPaused(p => !p)}>
            {isPaused ? <Play className="h-3 w-3 text-primary" /> : <Pause className="h-3 w-3" />}
          </Button>
          <div className="w-px h-4 bg-border/30 mx-0.5" />
          <Button variant="ghost" size="icon" className="h-6 w-6" onClick={() => exportCSV(data)} title="CSV">
            <FileText className="h-3 w-3" />
          </Button>
          <Button variant="ghost" size="icon" className="h-6 w-6" onClick={() => exportJSON(data)} title="Export JSON">
            <Download className="h-3 w-3" />
          </Button>
          <Button variant="ghost" size="icon" className="h-6 w-6" onClick={handleExportPNG} disabled={isExporting} title="Export PNG (current time range)">
            <Camera className={`h-3 w-3 ${isExporting ? "text-primary animate-pulse" : ""}`} />
          </Button>
        </div>
      </div>

      {/* ── Time window filter ──────────────────────────────────── */}
      <div className="flex items-center gap-1 px-3 py-1 border-b border-border/20 bg-card/40 flex-shrink-0 flex-wrap">
        <Clock className="h-2.5 w-2.5 text-muted-foreground mr-1" />
        {TIME_WINDOWS.map(tw => (
          <button key={tw.value} onClick={() => setTimeWindow(tw.value)}
            className={`px-1.5 py-0.5 rounded text-[8px] font-bold uppercase tracking-wider transition-colors
              ${timeWindow === tw.value
                ? "bg-primary/25 text-primary border border-primary/40"
                : "text-muted-foreground hover:text-foreground border border-transparent"}`}
          >
            {tw.label}
          </button>
        ))}
        {/* Custom range toggle */}
        <button onClick={handleSelectCustom}
          className={`px-1.5 py-0.5 rounded text-[8px] font-bold uppercase tracking-wider transition-colors
            ${timeWindow === "custom"
              ? "bg-primary/25 text-primary border border-primary/40"
              : "text-muted-foreground hover:text-foreground border border-transparent"}`}
        >
          CUSTOM
        </button>
        {/* From / To inputs — visible only when CUSTOM is active */}
        {timeWindow === "custom" && (
          <div className="flex items-center gap-1 ml-1">
            <span className="text-[8px] font-mono text-muted-foreground">FROM</span>
            <input
              type="datetime-local"
              step="1"
              value={customFrom}
              onChange={e => setCustomFrom(e.target.value)}
              className="h-5 px-1 text-[9px] bg-secondary/60 border border-border/40 rounded text-foreground focus:outline-none focus:border-primary/60 font-mono"
              style={{ colorScheme: "dark" }}
            />
            <span className="text-[8px] font-mono text-muted-foreground">TO</span>
            <input
              type="datetime-local"
              step="1"
              value={customTo}
              onChange={e => setCustomTo(e.target.value)}
              className="h-5 px-1 text-[9px] bg-secondary/60 border border-border/40 rounded text-foreground focus:outline-none focus:border-primary/60 font-mono"
              style={{ colorScheme: "dark" }}
            />
            {data.length > 0 && (
              <span className="text-[8px] font-mono text-muted-foreground/60">{data.length} rows</span>
            )}
          </div>
        )}
        {/* live server URL hint */}
        <span className="ml-auto text-[8px] font-mono text-muted-foreground/50 hidden sm:block">
          {DEBUG_API}
        </span>
      </div>

      {/* ── Chart grid ─────────────────────────────────────────── */}
      {/* overflow-y-auto plain div: removes Radix ScrollArea ResizeObserver + scrollbar DOM */}
      <ForceShowCtx.Provider value={isExporting}>
      <div ref={chartAreaRef} className="flex-1 overflow-y-auto">
        <div className="p-2 space-y-2">

          {/* Row 1: Drive Overview — render eagerly, always visible */}
          <Panel title="Drive Overview — Throttle PWM, Gear & Braking">
            {fullW > 0 && (
              <LineChart data={chartData} width={fullW} height={140}>
                <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                <XAxis dataKey="t" tick={tickStyle} stroke={C.axis}
                  label={{ value: "Sample", position: "insideBottom", offset: -2, style: { fontSize: 8, fill: C.axisText } }} />
                <YAxis tick={tickStyle} stroke={C.axis} />
                <Tooltip {...tooltipStyle} />
                <Line type="monotone"  dataKey="current_pwm"   stroke={C.teal}   strokeWidth={1.5} dot={false} isAnimationActive={false} name="PWM" />
                <Line type="stepAfter" dataKey="gas_pressed"   stroke={C.green}  strokeWidth={1}   dot={false} isAnimationActive={false} name="Gas" />
                <Line type="stepAfter" dataKey="brake_pressed" stroke={C.red}    strokeWidth={1}   dot={false} isAnimationActive={false} name="Brake" />
                <Line type="stepAfter" dataKey="is_braking"    stroke={C.orange} strokeWidth={1}   dot={false} isAnimationActive={false} name="Braking" />
              </LineChart>
            )}
          </Panel>

          {/* Row 2: Steering + Duty Cycles */}
          <LazyChart height={160} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Steering — Applied vs User Angle">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <ReferenceLine y={0} stroke={C.axis} strokeDasharray="3 3" />
                    <Line type="monotone" dataKey="steer_angle"      stroke={C.teal}   strokeWidth={1.5} dot={false} isAnimationActive={false} name="Steer" />
                    <Line type="monotone" dataKey="user_steer_angle" stroke={C.orange} strokeWidth={1}   dot={false} isAnimationActive={false} name="User" />
                  </LineChart>
                )}
              </Panel>
              <Panel title="Per-Wheel Duty Cycles">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="duty_fl" stroke={C.teal}   strokeWidth={1} dot={false} isAnimationActive={false} name="FL" />
                    <Line type="monotone" dataKey="duty_fr" stroke={C.green}  strokeWidth={1} dot={false} isAnimationActive={false} name="FR" />
                    <Line type="monotone" dataKey="duty_rl" stroke={C.orange} strokeWidth={1} dot={false} isAnimationActive={false} name="RL" />
                    <Line type="monotone" dataKey="duty_rr" stroke={C.red}    strokeWidth={1} dot={false} isAnimationActive={false} name="RR" />
                  </LineChart>
                )}
              </Panel>
            </div>
          </LazyChart>

          {/* Row 3: Wheel RPM + RPM Drift */}
          <LazyChart height={160} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Wheel RPM — Rear Left & Right">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="rpm_rear_left"   stroke={C.teal}   strokeWidth={1.5} dot={false} isAnimationActive={false} name="RL" />
                    <Line type="monotone" dataKey="rpm_rear_right"  stroke={C.green}  strokeWidth={1.5} dot={false} isAnimationActive={false} name="RR" />
                    <Line type="monotone" dataKey="rpm_front_right" stroke={C.orange} strokeWidth={1}   dot={false} isAnimationActive={false} name="FR" />
                  </LineChart>
                )}
              </Panel>
              <Panel title="RPM Drift (RR − RL)">
                {halfW > 0 && (
                  <BarChart data={driftData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <ReferenceLine y={0} stroke={C.axis} />
                    <Bar dataKey="drift" isAnimationActive={false}>
                      {driftData.map((d, i) => (
                        <Cell key={i} fill={d.drift >= 0 ? C.teal : C.red} fillOpacity={0.7} />
                      ))}
                    </Bar>
                  </BarChart>
                )}
              </Panel>
            </div>
          </LazyChart>

          {/* Row 4: Accelerometer + Gyroscope */}
          <LazyChart height={160} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Accelerometer (g)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} domain={[-0.1, 1.1]} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="accel_x" stroke={C.red}   strokeWidth={1} dot={false} isAnimationActive={false} name="X" />
                    <Line type="monotone" dataKey="accel_y" stroke={C.green} strokeWidth={1} dot={false} isAnimationActive={false} name="Y" />
                    <Line type="monotone" dataKey="accel_z" stroke={C.blue}  strokeWidth={1} dot={false} isAnimationActive={false} name="Z" />
                  </LineChart>
                )}
              </Panel>
              <Panel title="Gyroscope (°/s)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="gyro_x" stroke={C.red}   strokeWidth={1} dot={false} isAnimationActive={false} name="X" />
                    <Line type="monotone" dataKey="gyro_y" stroke={C.green} strokeWidth={1} dot={false} isAnimationActive={false} name="Y" />
                    <Line type="monotone" dataKey="gyro_z" stroke={C.blue}  strokeWidth={1} dot={false} isAnimationActive={false} name="Z" />
                  </LineChart>
                )}
              </Panel>
            </div>
          </LazyChart>

          {/* Row 5: Magnetometer + IMU Temperature */}
          <LazyChart height={160} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Magnetometer (Gauss)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="mag_x" stroke={C.red}    strokeWidth={1} dot={false} isAnimationActive={false} name="X" />
                    <Line type="monotone" dataKey="mag_y" stroke={C.green}  strokeWidth={1} dot={false} isAnimationActive={false} name="Y" />
                    <Line type="monotone" dataKey="mag_z" stroke={C.teal}   strokeWidth={1} dot={false} isAnimationActive={false} name="Z" />
                  </LineChart>
                )}
              </Panel>
              <Panel title="IMU Temperature (°C)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} domain={["dataMin - 0.5", "dataMax + 0.5"]} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="temp_c" stroke={C.yellow} strokeWidth={1.5} dot={false} isAnimationActive={false} name="Temp" />
                  </LineChart>
                )}
              </Panel>
            </div>
          </LazyChart>

          {/* Row 6: Compass Heading + PID Correction */}
          <LazyChart height={160} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Compass Heading (°)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="compass_heading"        stroke={C.teal}   strokeWidth={1.5} dot={false} isAnimationActive={false} name="Heading" />
                    <Line type="monotone" dataKey="compass_target_heading" stroke={C.purple} strokeWidth={1}   dot={false} isAnimationActive={false} strokeDasharray="4 2" name="Target" />
                  </LineChart>
                )}
              </Panel>
              <Panel title="Compass PID Correction (%)">
                {halfW > 0 && (
                  <BarChart data={pidData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <ReferenceLine y={0} stroke={C.axis} />
                    <Bar dataKey="pid" isAnimationActive={false}>
                      {pidData.map((d, i) => (
                        <Cell key={i} fill={d.pid >= 0 ? C.teal : C.orange} fillOpacity={0.8} />
                      ))}
                    </Bar>
                    <Line type="monotone" dataKey="err" stroke={C.yellow} strokeWidth={1} dot={false} isAnimationActive={false} name="Error" />
                  </BarChart>
                )}
              </Panel>
            </div>
          </LazyChart>

          {/* Row 7: Laser Distance (full width) */}
          <LazyChart height={172} scrollRoot={chartAreaRef}>
            <Panel title="Laser Distance (cm) — thresholds: 15 cm STOP / 25 CRAWL / 40 SLOW / 60 CAUTION">
              {fullW > 0 && (
                <LineChart data={chartData} width={fullW} height={140}>
                  <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                  <XAxis dataKey="t" tick={tickStyle} stroke={C.axis}
                    label={{ value: "Sample", position: "insideBottom", offset: -2, style: { fontSize: 8, fill: C.axisText } }} />
                  <YAxis tick={tickStyle} stroke={C.axis} />
                  <Tooltip {...tooltipStyle} />
                  <ReferenceLine y={15} stroke={C.red}    strokeDasharray="4 2" label={{ value: "STOP",    position: "right", style: { fontSize: 7, fill: C.red    } }} />
                  <ReferenceLine y={25} stroke={C.orange} strokeDasharray="4 2" label={{ value: "CRAWL",   position: "right", style: { fontSize: 7, fill: C.orange } }} />
                  <ReferenceLine y={40} stroke={C.yellow} strokeDasharray="4 2" label={{ value: "SLOW",    position: "right", style: { fontSize: 7, fill: C.yellow } }} />
                  <ReferenceLine y={60} stroke={C.green}  strokeDasharray="4 2" label={{ value: "CAUTION", position: "right", style: { fontSize: 7, fill: C.green  } }} />
                  <Line type="monotone" dataKey="laser_distance_cm" stroke={C.green} strokeWidth={2} dot={false} isAnimationActive={false} name="Distance" />
                </LineChart>
              )}
            </Panel>
          </LazyChart>

          {/* Row 8: Battery + Motor Current */}
          <LazyChart height={160} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Battery Voltage (V)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis tick={tickStyle} stroke={C.axis} domain={["dataMin - 0.05", "dataMax + 0.05"]} />
                    <Tooltip {...tooltipStyle} />
                    <Line type="monotone" dataKey="battery_voltage" stroke={C.yellow} strokeWidth={1.5} dot={false} isAnimationActive={false} name="Voltage" />
                  </LineChart>
                )}
              </Panel>
              <Panel title="Motor Current (A) + Power Limiter Max Duty (%)">
                {halfW > 0 && (
                  <LineChart data={chartData} width={halfW} height={120}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="t" tick={tickStyle} stroke={C.axis} />
                    <YAxis yAxisId="left"  tick={tickStyle} stroke={C.axis} />
                    <YAxis yAxisId="right" orientation="right" tick={tickStyle} stroke={C.axis} />
                    <Tooltip {...tooltipStyle} />
                    <Line yAxisId="left"  type="monotone" dataKey="current_amps"           stroke={C.orange} strokeWidth={1.5} dot={false} isAnimationActive={false} name="Current" />
                    <Line yAxisId="right" type="monotone" dataKey="power_limiter_max_duty" stroke={C.teal}   strokeWidth={1}   dot={false} isAnimationActive={false} strokeDasharray="3 2" name="Max Duty" />
                  </LineChart>
                )}
              </Panel>
            </div>
          </LazyChart>

          {/* Row 9: Mode Flags */}
          <LazyChart height={134} scrollRoot={chartAreaRef}>
            <Panel title="Mode Flags (event lane)">
              <div className="h-24 flex flex-col justify-center gap-1 px-1">
                {/* Obstacle State lane */}
                <div className="flex items-center gap-1">
                  <span className="text-[8px] font-mono text-muted-foreground w-16 text-right">STATE</span>
                  <div className="flex-1 flex h-5 rounded overflow-hidden">
                    {flagData.map((r, i) => (
                      <div key={i} className="flex-1 min-w-0"
                        style={{ backgroundColor: MODE_COLORS[r.obstacle_state] || C.grid }}
                        title={`t=${i}: ${r.obstacle_state}`} />
                    ))}
                  </div>
                </div>
                {(["autonomous_mode", "hunter_mode", "emergency_brake_active", "course_correction_active"] as const).map(field => (
                  <div key={field} className="flex items-center gap-1">
                    <span className="text-[8px] font-mono text-muted-foreground w-16 text-right truncate">
                      {field.replace(/_/g, " ").replace("active", "").trim()}
                    </span>
                    <div className="flex-1 flex h-3 rounded overflow-hidden">
                      {flagData.map((r, i) => (
                        <div key={i} className="flex-1 min-w-0"
                          style={{ backgroundColor: r[field] ? (field === "emergency_brake_active" ? C.red : C.teal) : C.grid }}
                          title={`t=${i}: ${r[field]}`} />
                      ))}
                    </div>
                  </div>
                ))}
                <div className="flex items-center gap-3 mt-1 px-16">
                  {Object.entries(MODE_COLORS).map(([state, color]) => (
                    <div key={state} className="flex items-center gap-1">
                      <span className="w-2 h-2 rounded-sm" style={{ backgroundColor: color }} />
                      <span className="text-[7px] font-mono text-muted-foreground">{state}</span>
                    </div>
                  ))}
                </div>
              </div>
            </Panel>
          </LazyChart>

          {/* Row 10: Scatter Plots */}
          <LazyChart height={172} scrollRoot={chartAreaRef}>
            <div className="grid grid-cols-2 gap-2">
              <Panel title="Magnetometer XY — calibration / hard-iron check">
                {halfW > 0 && (
                  <ScatterChart width={halfW} height={140}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="x" tick={tickStyle} stroke={C.axis} name="Mag X" type="number" />
                    <YAxis dataKey="y" tick={tickStyle} stroke={C.axis} name="Mag Y" type="number" />
                    <ZAxis dataKey="t" range={[15, 15]} />
                    <Tooltip {...tooltipStyle} />
                    <Scatter data={scatterMag} isAnimationActive={false}>
                      {scatterMag.map((_, i) => (
                        <Cell key={i} fill={`hsl(${(i / Math.max(scatterMag.length, 1)) * 300}, 80%, 55%)`} />
                      ))}
                    </Scatter>
                  </ScatterChart>
                )}
              </Panel>
              <Panel title="Accel XY scatter (lateral vs longitudinal)">
                {halfW > 0 && (
                  <ScatterChart width={halfW} height={140}>
                    <CartesianGrid strokeDasharray="2 2" stroke={C.grid} />
                    <XAxis dataKey="x" tick={tickStyle} stroke={C.axis} name="Accel X" type="number" />
                    <YAxis dataKey="y" tick={tickStyle} stroke={C.axis} name="Accel Y" type="number" />
                    <ZAxis dataKey="z" range={[10, 30]} />
                    <Tooltip {...tooltipStyle} />
                    <Scatter data={scatterAccel} fill={C.teal} fillOpacity={0.6} isAnimationActive={false} />
                  </ScatterChart>
                )}
              </Panel>
            </div>
          </LazyChart>

        </div>
      </div>
      </ForceShowCtx.Provider>

      {/* ── Status bar ─────────────────────────────────────────── */}
      <div className="h-5 flex items-center justify-between px-3 border-t border-border/30 bg-card/60 flex-shrink-0">
        <span className="text-[8px] font-mono text-muted-foreground">
          WINDOW: <span className="text-foreground">{timeWindow}</span>
          {" · "}SAMPLES: <span className="text-foreground">{data.length}</span>
          {" · "}API: <span className="text-foreground/60">{window.location.hostname}:5001</span>
        </span>
        <div className="flex items-center gap-1.5">
          {status === "connecting" && <RefreshCw className="h-2 w-2 text-yellow-400 animate-spin" />}
          <span className={`w-1.5 h-1.5 rounded-full ${
            isPaused          ? "bg-accent"
            : status === "live" ? "bg-primary animate-pulse"
            : status === "error" ? "bg-yellow-400"
            : "bg-zinc-500"
          }`} />
          <span className="text-[8px] font-mono text-muted-foreground">
            {isPaused ? "PAUSED" : status.toUpperCase()}
          </span>
        </div>
      </div>

    </div>
  );
};

export default Debug;
