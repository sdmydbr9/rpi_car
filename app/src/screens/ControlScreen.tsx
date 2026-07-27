import React, {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import {
  AppState,
  LayoutChangeEvent,
  PanResponder,
  SafeAreaView,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
  type ViewStyle,
} from "react-native";
import {
  ACTIVE_CONTROL_REFRESH_MS,
  isActiveControl,
  SPRING_RETURN_VALUE,
} from "../lib/manualInput";
import * as controls from "../lib/socketClient";
import type {
  ControlStatus,
  Direction,
  DriveInput,
  SteeringCalibration,
} from "../lib/socketClient";

const EMPTY_STATUS: ControlStatus = {
  pico_connected: false,
  armed: false,
  estop: false,
  active_source: null,
  remote_owner: false,
  gamepad_connected: false,
};

type AnalogPadProps = {
  kind: "throttle" | "steering";
  label: string;
  value: number;
  disabled: boolean;
  onChange: (value: number) => void;
};

function AnalogPad({
  kind,
  label,
  value,
  disabled,
  onChange,
}: AnalogPadProps) {
  const size = useRef({ width: 1, height: 1 });
  const setFromPosition = useCallback(
    (x: number, y: number) => {
      if (disabled) return;
      if (kind === "throttle") {
        const ratio = 1 - y / size.current.height;
        onChange(Math.round(Math.max(0, Math.min(1, ratio)) * 100));
      } else {
        const ratio = x / size.current.width;
        onChange(
          Math.round((Math.max(0, Math.min(1, ratio)) * 2 - 1) * 50),
        );
      }
    },
    [disabled, kind, onChange],
  );

  const responder = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => !disabled,
        onMoveShouldSetPanResponder: () => !disabled,
        onPanResponderGrant: (event) =>
          setFromPosition(
            event.nativeEvent.locationX,
            event.nativeEvent.locationY,
          ),
        onPanResponderMove: (event) =>
          setFromPosition(
            event.nativeEvent.locationX,
            event.nativeEvent.locationY,
          ),
        onPanResponderRelease: () => onChange(SPRING_RETURN_VALUE),
        onPanResponderTerminate: () => onChange(SPRING_RETURN_VALUE),
      }),
    [disabled, onChange, setFromPosition],
  );

  const onLayout = (event: LayoutChangeEvent) => {
    size.current = {
      width: event.nativeEvent.layout.width,
      height: event.nativeEvent.layout.height,
    };
  };

  const marker: ViewStyle =
    kind === "throttle"
      ? { left: "50%", bottom: `${value}%` as `${number}%` }
      : { left: `${value + 50}%` as `${number}%`, top: "50%" };

  return (
    <View style={styles.padColumn}>
      <View style={styles.padHeader}>
        <Text style={styles.padLabel}>{label}</Text>
        <Text style={styles.padValue}>
          {kind === "steering" && value > 0 ? "+" : ""}
          {value}
          {kind === "throttle" ? "%" : "°"}
        </Text>
      </View>
      <View
        style={[styles.pad, disabled && styles.disabled]}
        onLayout={onLayout}
        {...responder.panHandlers}
      >
        {kind === "steering" && <View style={styles.centerLine} />}
        <View style={[styles.padMarker, marker]} />
      </View>
      <Text style={styles.hint}>HOLD AND DRAG · RELEASE TO ZERO</Text>
    </View>
  );
}

export default function ControlScreen() {
  const [serverIp, setServerIp] = useState("192.168.4.1");
  const [connected, setConnected] = useState(false);
  const [ownsControl, setOwnsControl] = useState(false);
  const [status, setStatus] = useState(EMPTY_STATUS);
  const [message, setMessage] = useState("Connect to begin");
  const [battery, setBattery] = useState<number | null>(null);
  const [showCalibration, setShowCalibration] = useState(false);
  const [calibration, setCalibration] = useState<SteeringCalibration>({
    left_pw: 940,
    center_pw: 1440,
    right_pw: 2150,
  });
  const [drive, setDrive] = useState<Omit<DriveInput, "seq">>({
    direction: "N",
    throttle: 0,
    steering: 0,
    brake: false,
  });

  const sequence = useRef(0);
  const driveRef = useRef(drive);
  const statusRef = useRef(status);
  const ownedRef = useRef(false);

  useEffect(() => {
    driveRef.current = drive;
  }, [drive]);
  useEffect(() => {
    statusRef.current = status;
  }, [status]);
  useEffect(() => {
    ownedRef.current = ownsControl;
  }, [ownsControl]);

  const publishDrive = useCallback(
    (changes: Partial<Omit<DriveInput, "seq">>) => {
      const next = { ...driveRef.current, ...changes };
      driveRef.current = next;
      setDrive(next);
      if (ownedRef.current && statusRef.current.armed) {
        controls.sendDrive({ ...next, seq: ++sequence.current });
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
      controls.onStatus((next) => {
        setStatus(next);
        if (next.gamepad_connected) {
          setOwnsControl(false);
          setMessage("Gamepad has priority");
        }
        if (!next.armed) {
          driveRef.current = {
            ...driveRef.current,
            throttle: 0,
            brake: false,
          };
          setDrive(driveRef.current);
        }
      }),
      controls.onError(setMessage),
      controls.onBattery((result) => {
        if (result.status === "ok" && result.voltage !== undefined) {
          setBattery(result.voltage);
          setMessage(`Battery: ${result.voltage.toFixed(2)} V`);
        } else {
          setMessage(result.message || "Battery read failed");
        }
      }),
      controls.onCalibration(setCalibration),
    ];
    return () => unsubscribers.forEach((unsubscribe) => unsubscribe());
  }, [publishDrive]);

  useEffect(() => {
    if (
      !connected ||
      !ownsControl ||
      !status.armed ||
      !isActiveControl(drive)
    ) {
      return;
    }
    const timer = setInterval(() => {
      controls.sendDrive({
        ...driveRef.current,
        seq: ++sequence.current,
      });
    }, ACTIVE_CONTROL_REFRESH_MS);
    return () => clearInterval(timer);
  }, [connected, ownsControl, status.armed, drive.throttle, drive.brake]);

  useEffect(() => {
    const subscription = AppState.addEventListener("change", (state) => {
      if (state !== "active" && ownedRef.current) {
        controls.sendDrive({
          ...driveRef.current,
          throttle: 0,
          brake: false,
          seq: ++sequence.current,
        });
        controls.disarm();
      }
    });
    return () => subscription.remove();
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
    if (driveRef.current.throttle) {
      setMessage("Release throttle before changing direction");
      return;
    }
    publishDrive({ direction, throttle: 0 });
  };

  const setCalibrationField = (
    field: keyof SteeringCalibration,
    value: string,
  ) => {
    setCalibration((current) => ({
      ...current,
      [field]: Number.parseInt(value, 10) || 0,
    }));
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
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.container}>
        <View style={styles.header}>
          <View style={styles.titleBlock}>
            <Text style={styles.title}>MANUAL CAR CONTROL</Text>
            <Text style={styles.message}>{message}</Text>
          </View>
          <TextInput
            style={styles.ipInput}
            value={serverIp}
            editable={!connected}
            onChangeText={setServerIp}
            autoCapitalize="none"
          />
          <TouchableOpacity
            style={styles.primaryButton}
            onPress={connected ? controls.disconnect : connect}
          >
            <Text style={styles.darkButtonText}>
              {connected ? "DISCONNECT" : "CONNECT"}
            </Text>
          </TouchableOpacity>
        </View>

        <View style={styles.statusRow}>
          <Status label="PI" value={connected ? "ONLINE" : "OFFLINE"} />
          <Status label="PICO" value={status.pico_connected ? "READY" : "NO LINK"} />
          <Status
            label="CONTROL"
            value={
              status.gamepad_connected
                ? "GAMEPAD"
                : ownsControl
                  ? "THIS DEVICE"
                  : "UNCLAIMED"
            }
          />
          <Status label="ARM" value={status.armed ? "ARMED" : "SAFE"} />
          <Status
            label="BATTERY"
            value={battery === null ? "NOT READ" : `${battery.toFixed(2)} V`}
          />
        </View>

        <View style={styles.actionRow}>
          <TouchableOpacity
            disabled={!connected || status.gamepad_connected}
            style={[styles.outlineButton, (!connected || status.gamepad_connected) && styles.disabled]}
            onPress={ownsControl ? controls.releaseControl : controls.claimControl}
          >
            <Text style={styles.lightButtonText}>
              {ownsControl ? "RELEASE" : "TAKE CONTROL"}
            </Text>
          </TouchableOpacity>
          <TouchableOpacity
            disabled={!ownsControl || status.estop || status.gamepad_connected}
            style={[styles.armButton, (!ownsControl || status.estop || status.gamepad_connected) && styles.disabled]}
            onPress={() => {
              if (status.armed) {
                publishDrive({ direction: "N", throttle: 0, brake: false });
                controls.disarm();
              } else {
                controls.arm();
              }
            }}
          >
            <Text style={styles.darkButtonText}>
              {status.armed ? "DISARM" : "ARM"}
            </Text>
          </TouchableOpacity>
          <TouchableOpacity
            disabled={!connected || status.armed}
            style={[styles.outlineButton, (!connected || status.armed) && styles.disabled]}
            onPress={controls.readBattery}
          >
            <Text style={styles.lightButtonText}>READ BATTERY</Text>
          </TouchableOpacity>
          <TouchableOpacity
            disabled={estopDisabled}
            style={[styles.estopButton, status.estop && styles.estopReset, estopDisabled && styles.disabled]}
            onPress={
              status.estop
                ? controls.resetEmergencyStop
                : controls.emergencyStop
            }
          >
            <Text style={styles.lightButtonText}>
              {status.estop ? "RESET E-STOP" : "EMERGENCY STOP"}
            </Text>
          </TouchableOpacity>
        </View>

        <View style={styles.directionRow}>
          {(["R", "N", "F"] as Direction[]).map((direction) => (
            <TouchableOpacity
              key={direction}
              disabled={!enabled || drive.throttle !== 0}
              style={[
                styles.directionButton,
                drive.direction === direction && styles.directionActive,
                (!enabled || drive.throttle !== 0) && styles.disabled,
              ]}
              onPress={() => updateDirection(direction)}
            >
              <Text style={styles.directionText}>{direction}</Text>
            </TouchableOpacity>
          ))}
        </View>

        <View style={styles.pads}>
          <AnalogPad
            kind="steering"
            label="STEERING"
            value={drive.steering}
            disabled={!enabled}
            onChange={(steering) => publishDrive({ steering })}
          />
          <AnalogPad
            kind="throttle"
            label="THROTTLE"
            value={drive.throttle}
            disabled={!enabled || drive.direction === "N" || drive.brake}
            onChange={(throttle) => publishDrive({ throttle })}
          />
        </View>

        <TouchableOpacity
          disabled={!enabled}
          style={[styles.brakeButton, !enabled && styles.disabled]}
          onPressIn={() => publishDrive({ throttle: 0, brake: true })}
          onPressOut={() => publishDrive({ brake: false })}
        >
          <Text style={styles.brakeText}>HOLD BRAKE</Text>
        </TouchableOpacity>

        <TouchableOpacity
          disabled={status.armed}
          onPress={() => setShowCalibration((visible) => !visible)}
        >
          <Text style={styles.settingsLink}>CALIBRATION &amp; NETWORK</Text>
        </TouchableOpacity>
        {showCalibration && !status.armed && (
          <View style={styles.calibration}>
            {(["left_pw", "center_pw", "right_pw"] as const).map((field) => (
              <View key={field} style={styles.calibrationField}>
                <Text style={styles.calibrationLabel}>
                  {field.replace("_pw", "").toUpperCase()}
                </Text>
                <TextInput
                  style={styles.calibrationInput}
                  keyboardType="number-pad"
                  value={String(calibration[field])}
                  onChangeText={(value) => setCalibrationField(field, value)}
                />
              </View>
            ))}
            <TouchableOpacity
              disabled={!connected}
              style={[styles.primaryButton, !connected && styles.disabled]}
              onPress={() => controls.updateSteeringCalibration(calibration)}
            >
              <Text style={styles.darkButtonText}>APPLY</Text>
            </TouchableOpacity>
            <TouchableOpacity
              disabled={!connected}
              style={[styles.outlineButton, !connected && styles.disabled]}
              onPress={() => switchNetwork("wifi")}
            >
              <Text style={styles.lightButtonText}>WI-FI</Text>
            </TouchableOpacity>
            <TouchableOpacity
              disabled={!connected}
              style={[styles.outlineButton, !connected && styles.disabled]}
              onPress={() => switchNetwork("hotspot")}
            >
              <Text style={styles.lightButtonText}>HOTSPOT</Text>
            </TouchableOpacity>
          </View>
        )}
      </View>
    </SafeAreaView>
  );
}

function Status({ label, value }: { label: string; value: string }) {
  return (
    <View style={styles.status}>
      <Text style={styles.statusLabel}>{label}</Text>
      <Text style={styles.statusValue}>{value}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: "#020617" },
  container: { flex: 1, padding: 12, gap: 10 },
  header: { flexDirection: "row", alignItems: "center", gap: 8 },
  titleBlock: { flex: 1 },
  title: { color: "#67e8f9", fontSize: 20, fontWeight: "800" },
  message: { color: "#94a3b8", fontSize: 11 },
  ipInput: {
    width: 130,
    color: "#f8fafc",
    borderColor: "#334155",
    borderWidth: 1,
    borderRadius: 8,
    padding: 9,
  },
  primaryButton: {
    backgroundColor: "#22d3ee",
    borderRadius: 8,
    paddingHorizontal: 14,
    paddingVertical: 10,
    justifyContent: "center",
  },
  darkButtonText: { color: "#020617", fontWeight: "800", textAlign: "center" },
  lightButtonText: { color: "#f8fafc", fontWeight: "800", textAlign: "center" },
  statusRow: { flexDirection: "row", gap: 5 },
  status: {
    flex: 1,
    backgroundColor: "#0f172a",
    borderRadius: 7,
    padding: 7,
  },
  statusLabel: { color: "#64748b", fontSize: 9 },
  statusValue: { color: "#67e8f9", fontSize: 11, fontWeight: "700" },
  actionRow: { flexDirection: "row", gap: 7 },
  outlineButton: {
    flex: 1,
    borderColor: "#475569",
    borderWidth: 1,
    borderRadius: 8,
    padding: 10,
  },
  armButton: {
    flex: 1,
    backgroundColor: "#34d399",
    borderRadius: 8,
    padding: 10,
  },
  estopButton: {
    flex: 1.4,
    backgroundColor: "#dc2626",
    borderRadius: 8,
    padding: 10,
  },
  estopReset: { backgroundColor: "#d97706" },
  disabled: { opacity: 0.35 },
  directionRow: { flexDirection: "row", justifyContent: "center", gap: 9 },
  directionButton: {
    minWidth: 76,
    backgroundColor: "#0f172a",
    borderColor: "#334155",
    borderWidth: 1,
    borderRadius: 10,
    paddingVertical: 10,
  },
  directionActive: { backgroundColor: "#0891b2", borderColor: "#a5f3fc" },
  directionText: {
    color: "#f8fafc",
    textAlign: "center",
    fontSize: 20,
    fontWeight: "900",
  },
  pads: { flex: 1, flexDirection: "row", gap: 12 },
  padColumn: { flex: 1 },
  padHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 5,
  },
  padLabel: { color: "#94a3b8", fontSize: 11, fontWeight: "700" },
  padValue: { color: "#67e8f9", fontSize: 18, fontWeight: "800" },
  pad: {
    flex: 1,
    minHeight: 120,
    backgroundColor: "#020617",
    borderColor: "#155e75",
    borderWidth: 1,
    borderRadius: 16,
    overflow: "hidden",
  },
  centerLine: {
    position: "absolute",
    left: "50%",
    top: 0,
    bottom: 0,
    width: 1,
    backgroundColor: "#155e75",
  },
  padMarker: {
    position: "absolute",
    width: 28,
    height: 28,
    marginLeft: -14,
    marginTop: -14,
    marginBottom: -14,
    borderRadius: 14,
    borderColor: "#cffafe",
    borderWidth: 2,
    backgroundColor: "#22d3ee",
  },
  hint: {
    color: "#475569",
    fontSize: 8,
    textAlign: "center",
    marginTop: 4,
  },
  brakeButton: {
    backgroundColor: "#b91c1c",
    borderRadius: 12,
    paddingVertical: 14,
  },
  brakeText: {
    color: "#fff",
    fontSize: 18,
    fontWeight: "900",
    textAlign: "center",
    letterSpacing: 4,
  },
  settingsLink: {
    color: "#67e8f9",
    textAlign: "center",
    fontSize: 11,
    padding: 3,
  },
  calibration: { flexDirection: "row", gap: 7 },
  calibrationField: { flex: 1 },
  calibrationLabel: {
    color: "#64748b",
    fontSize: 8,
    textAlign: "center",
  },
  calibrationInput: {
    color: "#f8fafc",
    backgroundColor: "#0f172a",
    borderRadius: 7,
    padding: 9,
    textAlign: "center",
  },
});
