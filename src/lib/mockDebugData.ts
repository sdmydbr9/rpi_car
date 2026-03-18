/**
 * mockDebugData.ts
 * ────────────────
 * Type definition for a single debug ring-buffer row (mirrors
 * _DEBUG_LOG_FIELDS in scripts/main.py) and a generator for
 * dev / offline fallback mock data.
 */

export interface DebugSample {
  // time
  timestamp: string;
  // drive state
  gear: string;
  gas_pressed: number;
  brake_pressed: number;
  current_pwm: number;
  is_braking: number;
  steer_angle: number;
  user_steer_angle: number;
  obstacle_state: string;
  // speed encoders
  rpm_rear_right: number;
  rpm_rear_left: number;
  rpm_front_right: number;
  // accelerometer
  accel_x: number;
  accel_y: number;
  accel_z: number;
  // gyroscope
  gyro_x: number;
  gyro_y: number;
  gyro_z: number;
  // temperature
  temp_c: number;
  // magnetometer
  mag_x: number;
  mag_y: number;
  mag_z: number;
  // compass / PID
  compass_heading: number;
  compass_target_heading: number;
  pid_correction: number;
  heading_error_deg: number;
  steer_heading_delta_deg: number;
  course_correction_active: number;
  // laser
  laser_distance_cm: number;
  laser_raw_mm: number;
  // battery / power
  battery_voltage: number;
  current_amps: number;
  power_limiter_max_duty: number;
  power_limiter_l298n_drop: number;
  // motor duty cycles
  duty_fl: number;
  duty_fr: number;
  duty_rl: number;
  duty_rr: number;
  // autonomous state
  autonomous_mode: number;
  autonomous_state: string;
  hunter_mode: number;
  emergency_brake_active: number;
  // wheel sync
  sync_status: string;
}

/** Field names in the exact order pushed by _write_debug_row() */
export const DEBUG_FIELDS: (keyof DebugSample)[] = [
  "timestamp",
  "gear", "gas_pressed", "brake_pressed", "current_pwm", "is_braking",
  "steer_angle", "user_steer_angle", "obstacle_state",
  "rpm_rear_right", "rpm_rear_left", "rpm_front_right",
  "accel_x", "accel_y", "accel_z",
  "gyro_x", "gyro_y", "gyro_z",
  "temp_c",
  "mag_x", "mag_y", "mag_z",
  "compass_heading", "compass_target_heading", "pid_correction",
  "heading_error_deg", "steer_heading_delta_deg", "course_correction_active",
  "laser_distance_cm", "laser_raw_mm",
  "battery_voltage", "current_amps",
  "power_limiter_max_duty", "power_limiter_l298n_drop",
  "duty_fl", "duty_fr", "duty_rl", "duty_rr",
  "autonomous_mode", "autonomous_state", "hunter_mode",
  "emergency_brake_active",
  "sync_status",
];

/** Convert a server {fields, rows} snapshot into DebugSample objects */
export function rowsToSamples(fields: string[], rows: unknown[][]): DebugSample[] {
  return rows.map((row) => {
    const obj: Record<string, unknown> = {};
    fields.forEach((f, i) => { obj[f] = row[i]; });
    return obj as unknown as DebugSample;
  });
}

// ─── Mock data generator (offline / dev fallback) ────────────────

function lerp(a: number, b: number, t: number) { return a + (b - a) * t; }
function noise(amp: number) { return (Math.random() - 0.5) * 2 * amp; }

const STATES: DebugSample["obstacle_state"][] = [
  "IDLE","IDLE","IDLE","IDLE","IDLE","IDLE","IDLE","IDLE","IDLE","IDLE",
  "SLOW","SLOW","SLOW","SLOW","SLOW","SLOW","SLOW","SLOW",
  "CRAWL","CRAWL","CRAWL","CRAWL",
  "STOP","STOP","STOP",
  "AVOID","AVOID","AVOID",
  "IDLE","IDLE",
];

/**
 * Build N seconds worth of mock DebugSample data at 1 sample/second.
 * Used only when the live API is unreachable.
 */
export function buildMockDebugData(seconds = 30): DebugSample[] {
  const samples: DebugSample[] = [];
  const base = Date.now() - seconds * 1000;

  for (let i = 0; i < seconds; i++) {
    const t = i / seconds;
    const stateIdx = Math.min(i, STATES.length - 1);
    const state = STATES[stateIdx];
    const isMoving = state !== "STOP" && state !== "IDLE";
    const pwm = state === "IDLE" ? 0
      : state === "SLOW" ? lerp(30, 50, t) + noise(3)
      : state === "CRAWL" ? lerp(15, 25, t) + noise(2)
      : state === "AVOID" ? lerp(40, 60, t) + noise(5)
      : 0;
    const rpm = isMoving ? pwm * 1.8 + noise(5) : 0;
    const laser = state === "STOP" ? 12 + noise(1)
      : state === "CRAWL" ? 22 + noise(2)
      : state === "SLOW" ? 35 + noise(3)
      : 80 + noise(5);
    const heading = 180 + Math.sin(t * Math.PI * 2) * 30 + noise(2);
    const target = 180;
    const err = heading - target;

    samples.push({
      timestamp: new Date(base + i * 1000).toISOString(),
      gear: isMoving ? "D" : "P",
      gas_pressed: isMoving ? 1 : 0,
      brake_pressed: state === "STOP" ? 1 : 0,
      current_pwm: Math.max(0, pwm),
      is_braking: state === "STOP" ? 1 : 0,
      steer_angle: state === "AVOID" ? 45 : noise(3),
      user_steer_angle: noise(2),
      obstacle_state: state,
      rpm_rear_right: rpm + noise(2),
      rpm_rear_left: rpm + noise(2),
      rpm_front_right: rpm + noise(2),
      accel_x: noise(0.05),
      accel_y: noise(0.05),
      accel_z: 1.0 + noise(0.02),
      gyro_x: noise(0.5),
      gyro_y: noise(0.5),
      gyro_z: noise(1),
      temp_c: 42 + noise(0.5),
      mag_x: Math.cos((heading * Math.PI) / 180) * 0.35 + noise(0.01),
      mag_y: Math.sin((heading * Math.PI) / 180) * 0.35 + noise(0.01),
      mag_z: -0.15 + noise(0.005),
      compass_heading: heading,
      compass_target_heading: target,
      pid_correction: err * -0.01 + noise(0.003),
      heading_error_deg: err,
      steer_heading_delta_deg: err * 0.5,
      course_correction_active: Math.abs(err) > 5 ? 1 : 0,
      laser_distance_cm: Math.max(5, laser),
      laser_raw_mm: Math.max(50, laser * 10),
      battery_voltage: 7.4 - t * 0.1 + noise(0.01),
      current_amps: isMoving ? pwm * 0.05 + noise(0.05) : 0.1 + noise(0.02),
      power_limiter_max_duty: 95 - noise(1),
      power_limiter_l298n_drop: 1.8 + noise(0.05),
      duty_fl: Math.max(0, pwm + noise(2)),
      duty_fr: Math.max(0, pwm + noise(2)),
      duty_rl: Math.max(0, pwm + noise(2)),
      duty_rr: Math.max(0, pwm + noise(2)),
      autonomous_mode: 1,
      autonomous_state: state,
      hunter_mode: 0,
      emergency_brake_active: state === "STOP" ? 1 : 0,
      sync_status: isMoving ? "SYNC" : "OFF",
    });
  }
  return samples;
}
