import { io, Socket } from "socket.io-client";

export type Direction = "F" | "N" | "R";

export interface DriveInput {
  seq: number;
  direction: Direction;
  throttle: number;
  steering: number;
  brake: boolean;
}

export interface ControlStatus {
  pico_connected: boolean;
  pico_error?: string;
  armed: boolean;
  estop: boolean;
  active_source: "remote" | "gamepad" | null;
  remote_owner: boolean;
  gamepad_connected: boolean;
}

export interface SteeringCalibration {
  left_pw: number;
  center_pw: number;
  right_pw: number;
}

type Handlers = {
  connection: Set<(connected: boolean) => void>;
  status: Set<(status: ControlStatus) => void>;
  ownership: Set<(owned: boolean) => void>;
  error: Set<(message: string) => void>;
  battery: Set<(result: { status: string; voltage?: number; message?: string }) => void>;
  calibration: Set<(calibration: SteeringCalibration) => void>;
};

const handlers: Handlers = {
  connection: new Set(),
  status: new Set(),
  ownership: new Set(),
  error: new Set(),
  battery: new Set(),
  calibration: new Set(),
};

let socket: Socket | null = null;
let serverHttpUrl: string | null = null;

const notifyConnection = (connected: boolean) =>
  handlers.connection.forEach((handler) => handler(connected));
const notifyStatus = (status: ControlStatus) =>
  handlers.status.forEach((handler) => handler(status));
const notifyOwnership = (owned: boolean) =>
  handlers.ownership.forEach((handler) => handler(owned));
const notifyError = (message: string) =>
  handlers.error.forEach((handler) => handler(message));
const notifyBattery = (result: {
  status: string;
  voltage?: number;
  message?: string;
}) => handlers.battery.forEach((handler) => handler(result));
const notifyCalibration = (calibration: SteeringCalibration) =>
  handlers.calibration.forEach((handler) => handler(calibration));

export function connect(serverIp: string, port = 5000): Promise<void> {
  disconnect();
  const url = /^https?:\/\//.test(serverIp)
    ? serverIp.replace(/\/$/, "")
    : `http://${serverIp}:${port}`;
  serverHttpUrl = url;

  return new Promise((resolve, reject) => {
    socket = io(url, {
      transports: ["websocket", "polling"],
      reconnection: true,
      reconnectionDelay: 500,
      reconnectionDelayMax: 3000,
      timeout: 5000,
    });

    let settled = false;
    socket.on("connect", () => {
      notifyConnection(true);
      if (!settled) {
        settled = true;
        resolve();
      }
    });
    socket.on("disconnect", () => {
      notifyConnection(false);
      notifyOwnership(false);
    });
    socket.on("connect_error", (error) => {
      if (!settled) {
        settled = true;
        reject(error);
      }
    });
    socket.on("control_status", (status: ControlStatus) => {
      notifyStatus(status);
      if (status.gamepad_connected || !status.remote_owner) {
        notifyOwnership(false);
      }
    });
    socket.on("control_claim_result", (result: { owned: boolean }) => {
      notifyOwnership(!!result.owned);
    });
    socket.on("control_error", (data: { message?: string }) => {
      notifyError(data.message || "Control request rejected");
    });
    socket.on("battery_result", notifyBattery);
    socket.on(
      "steering_calibration",
      (data: { calibration: SteeringCalibration }) => {
        if (data?.calibration) notifyCalibration(data.calibration);
      },
    );
  });
}

export function disconnect(): void {
  if (socket) {
    socket.disconnect();
    socket = null;
  }
  notifyConnection(false);
  notifyOwnership(false);
}

export async function switchNetworkMode(
  mode: "wifi" | "hotspot",
): Promise<{ status: string; message?: string }> {
  if (!serverHttpUrl) {
    throw new Error("connect to the Raspberry Pi first");
  }
  const response = await fetch(`${serverHttpUrl}/system/switch_network_mode`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });
  const result = await response.json();
  if (!response.ok) {
    throw new Error(result.message || "network switch failed");
  }
  return result;
}

function send(event: string, payload: object = {}): void {
  if (socket?.connected) socket.emit(event, payload);
}

export const claimControl = () => send("control_claim");
export const releaseControl = () => send("control_release");
export const arm = () => send("arm");
export const disarm = () => send("disarm");
export const sendDrive = (input: DriveInput) => send("drive_input", input);
export const emergencyStop = () => send("emergency_stop");
export const resetEmergencyStop = () => send("emergency_stop_reset");
export const readBattery = () => send("battery_read");
export const updateSteeringCalibration = (calibration: SteeringCalibration) =>
  send("steering_calibration_update", calibration);

function subscribe<T>(set: Set<(value: T) => void>, callback: (value: T) => void) {
  set.add(callback);
  return () => set.delete(callback);
}

export const onConnection = (callback: (connected: boolean) => void) =>
  subscribe(handlers.connection, callback);
export const onStatus = (callback: (status: ControlStatus) => void) =>
  subscribe(handlers.status, callback);
export const onOwnership = (callback: (owned: boolean) => void) =>
  subscribe(handlers.ownership, callback);
export const onError = (callback: (message: string) => void) =>
  subscribe(handlers.error, callback);
export const onBattery = (
  callback: (result: { status: string; voltage?: number; message?: string }) => void,
) => subscribe(handlers.battery, callback);
export const onSteeringCalibration = (
  callback: (calibration: SteeringCalibration) => void,
) => subscribe(handlers.calibration, callback);
