import {
  type PointerEvent as ReactPointerEvent,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import {
  ACTIVE_CONTROL_REFRESH_MS,
  isActiveControl,
  SPRING_RETURN_VALUE,
} from "../../lib/manualInput";
import * as controls from "../../lib/socketClient";
import type {
  ControlStatus,
  Direction,
  DriveInput,
  SteeringCalibration,
} from "../../lib/socketClient";

const EMPTY_STATUS: ControlStatus = {
  pico_connected: false,
  armed: false,
  estop: false,
  active_source: null,
  remote_owner: false,
  gamepad_connected: false,
};

const DEFAULT_CALIBRATION: SteeringCalibration = {
  left_pw: 940,
  center_pw: 1440,
  right_pw: 2150,
};

type AnalogControlProps = {
  kind: "throttle" | "steering";
  label: string;
  value: number;
  disabled: boolean;
  onChange: (value: number) => void;
};

export function AnalogControl({
  kind,
  label,
  value,
  disabled,
  onChange,
}: AnalogControlProps) {
  const controlRef = useRef<HTMLDivElement>(null);

  const updateFromPointer = (event: ReactPointerEvent<HTMLDivElement>) => {
    if (disabled || !controlRef.current) return;
    const bounds = controlRef.current.getBoundingClientRect();
    if (kind === "throttle") {
      const ratio = 1 - (event.clientY - bounds.top) / bounds.height;
      onChange(Math.round(Math.max(0, Math.min(1, ratio)) * 100));
    } else {
      const ratio = (event.clientX - bounds.left) / bounds.width;
      onChange(Math.round((Math.max(0, Math.min(1, ratio)) * 2 - 1) * 50));
    }
  };

  const release = () => {
    if (!disabled) onChange(SPRING_RETURN_VALUE);
  };

  const markerStyle =
    kind === "throttle"
      ? { bottom: `${value}%`, left: "50%" }
      : { left: `${value + 50}%`, top: "50%" };

  return (
    <div className="flex min-h-0 flex-1 flex-col gap-3">
      <div className="flex items-center justify-between">
        <span className="text-xs font-semibold uppercase tracking-[0.2em] text-slate-400">
          {label}
        </span>
        <span className="font-mono text-xl text-cyan-300">
          {kind === "steering" && value > 0 ? "+" : ""}
          {value}
          {kind === "throttle" ? "%" : "°"}
        </span>
      </div>
      <div
        ref={controlRef}
        role="slider"
        aria-label={label}
        aria-valuemin={kind === "throttle" ? 0 : -50}
        aria-valuemax={kind === "throttle" ? 100 : 50}
        aria-valuenow={value}
        className={`relative min-h-36 flex-1 overflow-hidden rounded-2xl border border-cyan-400/30 bg-slate-950/80 touch-none ${
          disabled ? "cursor-not-allowed opacity-40" : "cursor-crosshair"
        }`}
        onPointerDown={(event) => {
          if (disabled) return;
          event.currentTarget.setPointerCapture(event.pointerId);
          updateFromPointer(event);
        }}
        onPointerMove={(event) => {
          if (event.currentTarget.hasPointerCapture(event.pointerId)) {
            updateFromPointer(event);
          }
        }}
        onPointerUp={(event) => {
          if (event.currentTarget.hasPointerCapture(event.pointerId)) {
            event.currentTarget.releasePointerCapture(event.pointerId);
          }
          release();
        }}
        onPointerCancel={release}
        onLostPointerCapture={release}
      >
        <div
          className={`absolute bg-cyan-300/15 ${
            kind === "throttle"
              ? "inset-x-0 bottom-0"
              : "inset-y-0 left-1/2 w-px bg-cyan-300/30"
          }`}
          style={kind === "throttle" ? { height: `${value}%` } : undefined}
        />
        <div
          className="absolute h-8 w-8 -translate-x-1/2 -translate-y-1/2 rounded-full border-2 border-cyan-200 bg-cyan-400 shadow-[0_0_24px_rgba(34,211,238,0.7)]"
          style={markerStyle}
        />
      </div>
      <p className="text-center text-[11px] uppercase tracking-wider text-slate-500">
        Hold and drag · release returns to zero
      </p>
    </div>
  );
}

export const CockpitController = () => {
  const suggestedHost = useMemo(() => {
    const host = window.location.hostname;
    return host && host !== "localhost" ? host : "localhost";
  }, []);
  const [serverIp, setServerIp] = useState(suggestedHost);
  const [connected, setConnected] = useState(false);
  const [ownsControl, setOwnsControl] = useState(false);
  const [status, setStatus] = useState<ControlStatus>(EMPTY_STATUS);
  const [message, setMessage] = useState("Connect to begin");
  const [battery, setBattery] = useState<number | null>(null);
  const [calibration, setCalibration] =
    useState<SteeringCalibration>(DEFAULT_CALIBRATION);
  const [showSettings, setShowSettings] = useState(false);
  const [drive, setDrive] = useState<Omit<DriveInput, "seq">>({
    direction: "N",
    throttle: 0,
    steering: 0,
    brake: false,
  });

  const driveRef = useRef(drive);
  const sequenceRef = useRef(0);
  const statusRef = useRef(status);
  const ownershipRef = useRef(false);

  useEffect(() => {
    driveRef.current = drive;
  }, [drive]);
  useEffect(() => {
    statusRef.current = status;
  }, [status]);
  useEffect(() => {
    ownershipRef.current = ownsControl;
  }, [ownsControl]);

  const publishDrive = useCallback(
    (changes: Partial<Omit<DriveInput, "seq">>) => {
      const next = { ...driveRef.current, ...changes };
      driveRef.current = next;
      setDrive(next);
      if (ownershipRef.current && statusRef.current.armed) {
        controls.sendDrive({ ...next, seq: ++sequenceRef.current });
      }
    },
    [],
  );

  useEffect(() => {
    const unsubscribers = [
      controls.onConnection((value) => {
        setConnected(value);
        if (!value) {
          setOwnsControl(false);
          setStatus(EMPTY_STATUS);
          driveRef.current = {
            direction: "N",
            throttle: 0,
            steering: 0,
            brake: false,
          };
          setDrive(driveRef.current);
        }
      }),
      controls.onOwnership((owned) => {
        setOwnsControl(owned);
        if (!owned) {
          driveRef.current = {
            ...driveRef.current,
            throttle: 0,
            brake: false,
          };
          setDrive(driveRef.current);
        }
      }),
      controls.onStatus((nextStatus) => {
        setStatus(nextStatus);
        if (nextStatus.gamepad_connected) {
          setOwnsControl(false);
          setMessage("Gamepad has priority");
        } else if (!nextStatus.armed) {
          driveRef.current = {
            ...driveRef.current,
            throttle: 0,
            brake: false,
          };
          setDrive(driveRef.current);
        }
      }),
      controls.onError((error) => setMessage(error)),
      controls.onBattery((result) => {
        if (result.status === "ok" && result.voltage !== undefined) {
          setBattery(result.voltage);
          setMessage(`Battery read: ${result.voltage.toFixed(2)} V`);
        } else {
          setMessage(result.message || "Battery read failed");
        }
      }),
      controls.onSteeringCalibration(setCalibration),
    ];
    return () => unsubscribers.forEach((unsubscribe) => unsubscribe());
  }, []);

  useEffect(() => {
    if (
      !connected ||
      !ownsControl ||
      !status.armed ||
      !isActiveControl(drive)
    ) {
      return;
    }
    const timer = window.setInterval(() => {
      const current = driveRef.current;
      controls.sendDrive({ ...current, seq: ++sequenceRef.current });
    }, ACTIVE_CONTROL_REFRESH_MS);
    return () => window.clearInterval(timer);
  }, [connected, ownsControl, status.armed, drive.throttle, drive.brake]);

  useEffect(() => {
    const stopForPageLoss = () => {
      if (document.visibilityState === "hidden" && ownershipRef.current) {
        controls.sendDrive({
          ...driveRef.current,
          throttle: 0,
          brake: false,
          seq: ++sequenceRef.current,
        });
        controls.disarm();
      }
    };
    window.addEventListener("pagehide", stopForPageLoss);
    document.addEventListener("visibilitychange", stopForPageLoss);
    return () => {
      window.removeEventListener("pagehide", stopForPageLoss);
      document.removeEventListener("visibilitychange", stopForPageLoss);
    };
  }, []);

  const connect = async () => {
    setMessage("Connecting…");
    try {
      await controls.connect(serverIp);
      setMessage("Connected. Take control to drive.");
    } catch {
      setMessage("Connection failed");
    }
  };

  const enabled =
    connected &&
    ownsControl &&
    status.armed &&
    !status.estop &&
    !status.gamepad_connected;
  const estopDisabled =
    !connected ||
    (status.estop && (!ownsControl || status.gamepad_connected));

  const updateDirection = (direction: Direction) => {
    if (driveRef.current.throttle !== 0) {
      setMessage("Release throttle before changing direction");
      return;
    }
    publishDrive({ direction, throttle: 0 });
  };

  const toggleArm = () => {
    if (status.armed) {
      publishDrive({ throttle: 0, brake: false, direction: "N" });
      controls.disarm();
    } else {
      controls.arm();
    }
  };

  const applyCalibration = () => {
    controls.updateSteeringCalibration(calibration);
    setMessage("Steering calibration sent");
  };

  const switchNetwork = async (mode: "wifi" | "hotspot") => {
    if (status.armed) {
      setMessage("Disarm before switching networks");
      return;
    }
    try {
      await controls.switchNetworkMode(mode);
      setMessage(`Switching to ${mode}; reconnect shortly`);
    } catch (error) {
      setMessage(
        error instanceof Error ? error.message : "Network switch failed",
      );
    }
  };

  return (
    <main className="flex h-[100dvh] w-full flex-col bg-slate-950 p-3 text-slate-100 sm:p-5">
      <header className="mb-3 flex flex-wrap items-center gap-3 rounded-2xl border border-slate-800 bg-slate-900/80 p-3">
        <div className="mr-auto">
          <h1 className="text-lg font-bold uppercase tracking-[0.18em] text-cyan-300 sm:text-2xl">
            Manual Car Control
          </h1>
          <p className="text-xs text-slate-400">{message}</p>
        </div>
        <input
          value={serverIp}
          disabled={connected}
          onChange={(event) => setServerIp(event.target.value)}
          className="w-40 rounded-lg border border-slate-700 bg-slate-950 px-3 py-2 text-sm"
          aria-label="Raspberry Pi address"
        />
        <button
          onClick={connected ? controls.disconnect : connect}
          className="rounded-lg bg-cyan-500 px-4 py-2 text-sm font-bold text-slate-950"
        >
          {connected ? "Disconnect" : "Connect"}
        </button>
        <button
          onClick={() => setShowSettings((open) => !open)}
          className="rounded-lg border border-slate-700 px-4 py-2 text-sm"
        >
          Settings
        </button>
      </header>

      <section className="mb-3 grid grid-cols-2 gap-2 text-xs sm:grid-cols-5">
        <StatusChip label="Network" value={connected ? "Connected" : "Offline"} ok={connected} />
        <StatusChip
          label="Pico"
          value={status.pico_connected ? "Ready" : "Unavailable"}
          ok={status.pico_connected}
        />
        <StatusChip
          label="Controller"
          value={status.gamepad_connected ? "Gamepad" : ownsControl ? "This device" : "Unclaimed"}
          ok={ownsControl || status.gamepad_connected}
        />
        <StatusChip label="Arm" value={status.armed ? "Armed" : "Disarmed"} ok={status.armed} />
        <StatusChip label="Battery" value={battery === null ? "Not read" : `${battery.toFixed(2)} V`} ok={battery !== null} />
      </section>

      {showSettings && (
        <section className="mb-3 grid gap-3 rounded-2xl border border-slate-700 bg-slate-900 p-4 sm:grid-cols-2">
          <div>
            <h2 className="mb-2 font-bold uppercase tracking-wider text-cyan-300">
              Steering calibration
            </h2>
            <div className="grid grid-cols-3 gap-2">
              {(["left_pw", "center_pw", "right_pw"] as const).map((field) => (
                <label key={field} className="text-xs text-slate-400">
                  {field.replace("_pw", "")}
                  <input
                    type="number"
                    value={calibration[field]}
                    disabled={!connected || status.armed}
                    onChange={(event) =>
                      setCalibration((current) => ({
                        ...current,
                        [field]: Number(event.target.value),
                      }))
                    }
                    className="mt-1 w-full rounded border border-slate-700 bg-slate-950 p-2 text-slate-100"
                  />
                </label>
              ))}
            </div>
            <button
              disabled={!connected || status.armed}
              onClick={applyCalibration}
              className="mt-3 rounded bg-cyan-500 px-4 py-2 text-sm font-bold text-slate-950 disabled:opacity-40"
            >
              Apply calibration
            </button>
          </div>
          <div>
            <h2 className="mb-2 font-bold uppercase tracking-wider text-cyan-300">
              Network
            </h2>
            <p className="mb-3 text-xs text-slate-400">
              Switching is allowed only while disarmed and will disconnect this page.
            </p>
            <div className="flex gap-2">
              <button disabled={!connected || status.armed} onClick={() => switchNetwork("wifi")} className="rounded border border-slate-600 px-4 py-2 disabled:opacity-40">
                Wi-Fi
              </button>
              <button disabled={!connected || status.armed} onClick={() => switchNetwork("hotspot")} className="rounded border border-slate-600 px-4 py-2 disabled:opacity-40">
                Hotspot
              </button>
            </div>
          </div>
        </section>
      )}

      <section className="mb-3 flex flex-wrap items-center gap-2 rounded-2xl border border-slate-800 bg-slate-900/70 p-3">
        {!ownsControl ? (
          <button
            disabled={!connected || status.gamepad_connected}
            onClick={controls.claimControl}
            className="rounded-lg bg-cyan-500 px-4 py-2 font-bold text-slate-950 disabled:opacity-40"
          >
            Take control
          </button>
        ) : (
          <button
            disabled={status.armed}
            onClick={controls.releaseControl}
            className="rounded-lg border border-slate-600 px-4 py-2 disabled:opacity-40"
          >
            Release control
          </button>
        )}
        <button
          disabled={!ownsControl || status.estop || status.gamepad_connected}
          onClick={toggleArm}
          className={`rounded-lg px-5 py-2 font-bold disabled:opacity-40 ${
            status.armed ? "bg-amber-500 text-slate-950" : "bg-emerald-500 text-slate-950"
          }`}
        >
          {status.armed ? "Disarm" : "Arm"}
        </button>
        <button
          disabled={!connected || status.armed}
          onClick={controls.readBattery}
          className="rounded-lg border border-slate-600 px-4 py-2 disabled:opacity-40"
        >
          Read battery
        </button>
        <button
          disabled={estopDisabled}
          onClick={status.estop ? controls.resetEmergencyStop : controls.emergencyStop}
          className={`ml-auto rounded-lg px-6 py-3 font-black uppercase ${
            status.estop ? "bg-amber-400 text-slate-950" : "bg-red-600 text-white"
          } disabled:opacity-40`}
        >
          {status.estop ? "Reset E-stop" : "Emergency stop"}
        </button>
      </section>

      <section className="mb-3 flex items-center justify-center gap-2">
        {(["R", "N", "F"] as Direction[]).map((direction) => (
          <button
            key={direction}
            disabled={!enabled || drive.throttle !== 0}
            onClick={() => updateDirection(direction)}
            className={`min-w-20 rounded-xl border px-6 py-3 text-xl font-black disabled:opacity-35 ${
              drive.direction === direction
                ? direction === "R"
                  ? "border-red-300 bg-red-600"
                  : "border-cyan-200 bg-cyan-500 text-slate-950"
                : "border-slate-700 bg-slate-900"
            }`}
          >
            {direction}
          </button>
        ))}
      </section>

      <section className="flex min-h-0 flex-1 gap-3">
        <AnalogControl
          kind="steering"
          label="Steering"
          value={drive.steering}
          disabled={!enabled}
          onChange={(steering) => publishDrive({ steering })}
        />
        <AnalogControl
          kind="throttle"
          label="Throttle"
          value={drive.throttle}
          disabled={!enabled || drive.direction === "N" || drive.brake}
          onChange={(throttle) => publishDrive({ throttle })}
        />
      </section>

      <button
        disabled={!enabled}
        onPointerDown={() => publishDrive({ throttle: 0, brake: true })}
        onPointerUp={() => publishDrive({ brake: false })}
        onPointerCancel={() => publishDrive({ brake: false })}
        onPointerLeave={() => {
          if (driveRef.current.brake) publishDrive({ brake: false });
        }}
        className="mt-3 rounded-2xl bg-red-700 py-4 text-xl font-black uppercase tracking-[0.3em] disabled:opacity-40"
      >
        Hold brake
      </button>
    </main>
  );
};

function StatusChip({
  label,
  value,
  ok,
}: {
  label: string;
  value: string;
  ok: boolean;
}) {
  return (
    <div className="rounded-lg border border-slate-800 bg-slate-900 px-3 py-2">
      <span className="block uppercase tracking-wider text-slate-500">{label}</span>
      <span className={ok ? "text-cyan-300" : "text-slate-300"}>{value}</span>
    </div>
  );
}
