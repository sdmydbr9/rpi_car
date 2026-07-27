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

let socket: Socket | null = null;
let serverHttpUrl: string | null = null;
const connectionHandlers = new Set<(connected: boolean) => void>();
const statusHandlers = new Set<(status: ControlStatus) => void>();
const ownershipHandlers = new Set<(owned: boolean) => void>();
const errorHandlers = new Set<(message: string) => void>();
const batteryHandlers = new Set<
  (result: { status: string; voltage?: number; message?: string }) => void
>();
const calibrationHandlers = new Set<
  (calibration: SteeringCalibration) => void
>();

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
      connectionHandlers.forEach((handler) => handler(true));
      if (!settled) {
        settled = true;
        resolve();
      }
    });
    socket.on("disconnect", () => {
      connectionHandlers.forEach((handler) => handler(false));
      ownershipHandlers.forEach((handler) => handler(false));
    });
    socket.on("connect_error", (error) => {
      if (!settled) {
        settled = true;
        reject(error);
      }
    });
    socket.on("control_status", (status: ControlStatus) => {
      statusHandlers.forEach((handler) => handler(status));
      if (status.gamepad_connected || !status.remote_owner) {
        ownershipHandlers.forEach((handler) => handler(false));
      }
    });
    socket.on("control_claim_result", ({ owned }: { owned: boolean }) => {
      ownershipHandlers.forEach((handler) => handler(!!owned));
    });
    socket.on("control_error", ({ message }: { message?: string }) => {
      errorHandlers.forEach((handler) =>
        handler(message || "Control request rejected"),
      );
    });
    socket.on("battery_result", (result) => {
      batteryHandlers.forEach((handler) => handler(result));
    });
    socket.on(
      "steering_calibration",
      ({ calibration }: { calibration: SteeringCalibration }) => {
        if (calibration) {
          calibrationHandlers.forEach((handler) => handler(calibration));
        }
      },
    );
  });
}

export function disconnect(): void {
  socket?.disconnect();
  socket = null;
  connectionHandlers.forEach((handler) => handler(false));
  ownershipHandlers.forEach((handler) => handler(false));
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

function emit(event: string, payload: object = {}) {
  if (socket?.connected) socket.emit(event, payload);
}

export const claimControl = () => emit("control_claim");
export const releaseControl = () => emit("control_release");
export const arm = () => emit("arm");
export const disarm = () => emit("disarm");
export const sendDrive = (input: DriveInput) => emit("drive_input", input);
export const emergencyStop = () => emit("emergency_stop");
export const resetEmergencyStop = () => emit("emergency_stop_reset");
export const readBattery = () => emit("battery_read");
export const updateSteeringCalibration = (calibration: SteeringCalibration) =>
  emit("steering_calibration_update", calibration);

function subscribe<T>(set: Set<(value: T) => void>, callback: (value: T) => void) {
  set.add(callback);
  return () => set.delete(callback);
}

export const onConnection = (callback: (connected: boolean) => void) =>
  subscribe(connectionHandlers, callback);
export const onStatus = (callback: (status: ControlStatus) => void) =>
  subscribe(statusHandlers, callback);
export const onOwnership = (callback: (owned: boolean) => void) =>
  subscribe(ownershipHandlers, callback);
export const onError = (callback: (message: string) => void) =>
  subscribe(errorHandlers, callback);
export const onBattery = (
  callback: (result: { status: string; voltage?: number; message?: string }) => void,
) => subscribe(batteryHandlers, callback);
export const onCalibration = (
  callback: (calibration: SteeringCalibration) => void,
) => subscribe(calibrationHandlers, callback);
