import { useState, useEffect, useCallback, useRef } from "react";
import { subscribeTelemetry, emitOdometryReset, type TelemetryData } from "@/lib/socketClient";

export interface OdometryPoint {
  x: number;
  y: number;
  heading: number;
  timestamp: number;
}

export interface OdometryData {
  x: number;
  y: number;
  heading: number;
  vLinear: number;
  vAngular: number;
  active: boolean;
}

interface UseOdometryOptions {
  maxTrailPoints?: number;
  minMovementThreshold?: number; // meters
}

export function useOdometry(options: UseOdometryOptions = {}) {
  const {
    maxTrailPoints = 2000,
    minMovementThreshold = 0.003, // 3mm — fine-grained for smooth path
  } = options;

  const [odometry, setOdometry] = useState<OdometryData>({
    x: 0,
    y: 0,
    heading: 0,
    vLinear: 0,
    vAngular: 0,
    active: false,
  });

  const [trail, setTrail] = useState<OdometryPoint[]>([
    { x: 0, y: 0, heading: 0, timestamp: Date.now() },
  ]);

  const lastRecordedPoint = useRef<OdometryPoint>({
    x: 0,
    y: 0,
    heading: 0,
    timestamp: Date.now(),
  });

  const [rthActive, setRthActive] = useState(false);

  const resetTrail = useCallback(() => {
    const origin: OdometryPoint = {
      x: 0,
      y: 0,
      heading: 0,
      timestamp: Date.now(),
    };
    setTrail([origin]);
    lastRecordedPoint.current = origin;
    setOdometry((prev) => ({ ...prev, x: 0, y: 0, heading: 0 }));
    emitOdometryReset();
  }, []);

  // Subscribe to real telemetry from Socket.IO
  useEffect(() => {
    const unsub = subscribeTelemetry((data: TelemetryData) => {
      const active = data.odometry_active ?? false;
      const x = data.odometry_x_m ?? 0;
      const y = data.odometry_y_m ?? 0;
      const heading = data.odometry_heading_deg ?? 0;
      const vLinear = data.odometry_v_linear ?? 0;
      const vAngular = data.odometry_v_angular ?? 0;

      setOdometry({ x, y, heading, vLinear, vAngular, active });
      setRthActive(data.return_to_start_active ?? false);

      if (!active) return;

      // Use server-side trail when available. The backend TrailRecorder
      // accumulates points from the fused odometry engine and sends them
      // with every telemetry update. This survives page reloads and
      // ensures the map always shows the authoritative recorded path.
      const serverTrail = data.odometry_trail;
      if (serverTrail && serverTrail.length > 0) {
        const now = Date.now();
        const synced: OdometryPoint[] = serverTrail.map((p) => ({
          x: p.x,
          y: p.y,
          heading: 0,
          timestamp: now,
        }));
        // Append current live position as the latest point so the path
        // extends to exactly where the rover is right now.
        synced.push({ x, y, heading, timestamp: now });
        const capped = synced.length > maxTrailPoints ? synced.slice(-maxTrailPoints) : synced;
        setTrail(capped);
        lastRecordedPoint.current = { x, y, heading, timestamp: now };
        return;
      }

      // Fallback: client-side accumulation when the server trail is
      // empty (odometry just started or no movement recorded yet).
      const last = lastRecordedPoint.current;
      const dx = x - last.x;
      const dy = y - last.y;
      const dist = Math.sqrt(dx * dx + dy * dy);

      if (dist >= minMovementThreshold) {
        const point: OdometryPoint = { x, y, heading, timestamp: Date.now() };
        lastRecordedPoint.current = point;

        setTrail((prev) => {
          const next = [...prev, point];
          return next.length > maxTrailPoints
            ? next.slice(next.length - maxTrailPoints)
            : next;
        });
      }
    });
    return unsub;
  }, [maxTrailPoints, minMovementThreshold]);

  const distanceFromStart = Math.sqrt(
    odometry.x * odometry.x + odometry.y * odometry.y
  );

  return {
    odometry,
    trail,
    distanceFromStart,
    resetTrail,
    rthActive,
  };
}
