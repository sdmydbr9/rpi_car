# pico_sensor_bridge.py — Pico W Dual-Core Sensor + Motor Controller
#
# RP2040 Dual-Core Architecture:
# ┌─────────────────────────────────────────────────┐
# │  Core 1 — Fast Loop (100 Hz)                     │
# │  • Motor PWM ramp tick         every 10 ms       │
# │  • RPM computation (from PIO)  every 10 ms       │
# │  • Motor safety watchdog       every 10 ms       │
# └─────────────────────────────────────────────────┘
# ┌─────────────────────────────────────────────────┐
# │  Core 0 — Main Loop (~25-50 Hz)                 │
# │  • I2C sensor reads (IMU, laser, mag, ADC)      │
# │  • UART RX command parser      each iteration    │
# │  • UART TX JSON telemetry      each iteration    │
# │  • LED heartbeat blink                           │
# └─────────────────────────────────────────────────┘
#
# PIO state machines decode quadrature encoders in hardware —
# zero CPU load, zero missed edges at any motor speed.
#
# Chassis: 2WD rear-drive with Ackermann front steering servo

from machine import Pin, I2C, UART, PWM
import machine
import rp2
import _thread
import time
import struct
import json
import gc

print("\n" + "="*60)
print("  PICO W SENSOR BRIDGE — Dual-Core + PIO Encoders")
print("  Core 0: Sensors @ 50 Hz | Core 1: Motors+RPM @ 100 Hz")
print("="*60 + "\n")

# =====================================================
# LED SETUP (Pico W built-in LED)
# =====================================================
led = Pin("LED", Pin.OUT)
led.off()

# =====================================================
# UART SETUP (to Raspberry Pi)
# =====================================================
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), txbuf=512)
print("✅ UART0 initialized: TX=GPIO0, RX=GPIO1 @ 115200 baud (txbuf=512)")

# =====================================================
# PIO ENCODER (verified working method)
# =====================================================
# PIO counts rising edges on pin_a, using pin_b (jmp_pin) for
# direction. The running count is kept in the X register —
# read on demand via exec("in_(x,32)") + push/get.
# No FIFO drain loop needed.
#
# Pin wiring (physically tested):
#   Left:  Phase A = GP3, Phase B = GP2
#   Right: Phase A = GP4, Phase B = GP5
# =====================================================

@rp2.asm_pio()
def pio_encoder():
    wrap_target()
    wait(1, pin, 0)
    jmp(pin, "count_up")

    jmp(x_dec, "wait_low")

    label("count_up")
    mov(x, invert(x))
    jmp(x_dec, "re_invert")
    label("re_invert")
    mov(x, invert(x))

    label("wait_low")
    wait(0, pin, 0)
    wrap()


class PIOMotorEncoder:
    """PIO encoder that keeps a running count in the X register.
    Direction is determined by jmp_pin (phase B).
    Read count on demand — no FIFO drain loop needed."""

    def __init__(self, sm_id, pin_a_num, pin_b_num, name="enc"):
        self.name = name
        self.pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)

        self.sm = rp2.StateMachine(
            sm_id, pio_encoder, freq=10_000_000,
            in_base=self.pin_a, jmp_pin=self.pin_b
        )
        self.sm.exec("set(x, 0)")
        self.sm.active(1)

        self.zero_offset = 0

    def get_raw_count(self):
        """Read absolute hardware count from PIO X register."""
        self.sm.exec("in_(x, 32)")
        self.sm.exec("push()")
        val = self.sm.get()
        if val > (1 << 31):
            val -= (1 << 32)
        return val

    def get_position(self):
        """Count relative to last reset."""
        return self.get_raw_count() - self.zero_offset

    def reset_position(self):
        """Zero the position by snapping offset to current count."""
        self.zero_offset = self.get_raw_count()


# Instantiate PIO encoders — SM 0 and 1
enc_left  = PIOMotorEncoder(0, 3, 2, "left")    # SM0: GP3(A)/GP2(B)
enc_right = PIOMotorEncoder(1, 4, 5, "right")   # SM1: GP4(A)/GP5(B)
print("✅ PIO encoders initialized: Left(GP3/GP2 SM0), Right(GP4/GP5 SM1)")

# ── RPM Configuration ──
ENCODER_PPR = 330.0                   # Pulses per revolution
ALPHA_RPM = 0.3                       # EMA filter coefficient

# ── Shared state (written by Core 1, read by Core 0 for UART TX) ──
_rpm_left  = 0.0
_rpm_right = 0.0
_rpm_filtered_left  = 0.0
_rpm_filtered_right = 0.0
_enc_pos_left  = 0
_enc_pos_right = 0

# =====================================================
# SERVO HELPERS
# =====================================================
SERVO_MIN_DUTY = 2500
SERVO_MAX_DUTY = 7500

def _servo_clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def _set_servo(pwm, angle):
    angle = _servo_clamp(angle, 0, 180)
    duty = int(SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * (angle / 180.0))
    pwm.duty_u16(duty)

# =====================================================
# PAN-TILT SERVO SETUP (PWM on GP18 + GP19)
# Calibrated values from field testing with camera module
# =====================================================
pan_servo = PWM(Pin(18))
tilt_servo = PWM(Pin(19))
pan_servo.freq(50)
tilt_servo.freq(50)

# Calibrated PWM duty cycles (tested with camera servo)
PAN_TILT_MIN_DUTY = 1638      # 0° position
PAN_TILT_MAX_DUTY = 7864      # 180° position
PAN_CENTER  = 90;  TILT_CENTER = 90
PAN_MIN  = 0;   PAN_MAX  = 180
TILT_MIN = 45;  TILT_MAX = 135

_pan_angle  = PAN_CENTER
_tilt_angle = TILT_CENTER

def _set_pan_tilt_servo(pwm, angle):
    """Set servo angle using calibrated PWM duty cycle."""
    angle = max(0, min(180, angle))
    duty = int(PAN_TILT_MIN_DUTY + (PAN_TILT_MAX_DUTY - PAN_TILT_MIN_DUTY) * (angle / 180.0))
    pwm.duty_u16(duty)

def _apply_pan_tilt():
    _set_pan_tilt_servo(pan_servo, _pan_angle)
    _set_pan_tilt_servo(tilt_servo, _tilt_angle)

_apply_pan_tilt()
print(f"✅ Pan-tilt servos: GP18 (pan:{PAN_MIN}–{PAN_MAX}°), GP19 (tilt:{TILT_MIN}–{TILT_MAX}°) — centered at {PAN_CENTER}°")

# =====================================================
# ACKERMANN STEERING SERVO (PWM on GP15)
# The Pi now owns the chassis-specific left/center/right calibration.
# Keep only a broad low-level clamp here so future calibration changes
# do not require reflashing the Pico.
# =====================================================
steer_servo = PWM(Pin(15))
steer_servo.freq(50)

STEER_DEFAULT_CENTER_PW = 1440
STEER_ABS_MIN_PW = 500
STEER_ABS_MAX_PW = 2500
_current_steer_pw = STEER_DEFAULT_CENTER_PW

def set_steering_pw(pw_us):
    global _current_steer_pw
    pw_us = _servo_clamp(pw_us, STEER_ABS_MIN_PW, STEER_ABS_MAX_PW)
    _current_steer_pw = pw_us
    steer_servo.duty_ns(pw_us * 1000)

set_steering_pw(STEER_DEFAULT_CENTER_PW)
print(
    "✅ Steering servo: GP15 — default center "
    f"{STEER_DEFAULT_CENTER_PW}µs [abs {STEER_ABS_MIN_PW}–{STEER_ABS_MAX_PW}]"
)

# =====================================================
# L298N MOTOR DRIVER (2WD rear drive)
#   Left:  ENA=GP10(PWM), IN1=GP17, IN2=GP12
#   Right: ENB=GP16(PWM), IN3=GP13, IN4=GP14
# =====================================================
mot_ena = PWM(Pin(10)); mot_ena.freq(1000)
mot_in1 = Pin(17, Pin.OUT)
mot_in2 = Pin(12, Pin.OUT)
mot_in3 = Pin(13, Pin.OUT)
mot_in4 = Pin(14, Pin.OUT)
mot_enb = PWM(Pin(16)); mot_enb.freq(1000)

_motor_target_duty_l = 0
_motor_target_duty_r = 0
_motor_applied_duty_l = 0.0
_motor_applied_duty_r = 0.0
_motor_forward = True
_last_motor_fwd = None
_last_cmd_time = time.ticks_ms()

# At 100 Hz motor tick (10 ms), 3% per tick = 300%/sec ramp
MAX_PWM_DELTA_PER_TICK = 3
DIRECTION_CHANGE_DWELL_MS = 150
WATCHDOG_TIMEOUT_MS = 500

# Non-blocking brake state machine
_brake_active = False
_brake_start_time = 0
_brake_start_duty_l = 0
_brake_start_duty_r = 0

# Non-blocking direction-change dwell
_dwell_active = False
_dwell_start_time = 0

def _set_motor_pins(forward):
    if forward:
        mot_in1.value(0); mot_in2.value(1)
        mot_in3.value(0); mot_in4.value(1)
    else:
        mot_in1.value(1); mot_in2.value(0)
        mot_in3.value(1); mot_in4.value(0)

def _set_motor_pwm_l(duty_pct):
    d = int(duty_pct / 100.0 * 65535)
    d = max(0, min(65535, d))
    mot_ena.duty_u16(d)

def _set_motor_pwm_r(duty_pct):
    d = int(duty_pct / 100.0 * 65535)
    d = max(0, min(65535, d))
    mot_enb.duty_u16(d)

def _set_motor_pwm(duty_pct):
    _set_motor_pwm_l(duty_pct)
    _set_motor_pwm_r(duty_pct)

def _ramp_one(applied, target):
    """Ramp a single motor duty one tick toward target. Returns new applied."""
    delta = target - applied
    if abs(delta) > MAX_PWM_DELTA_PER_TICK:
        delta = MAX_PWM_DELTA_PER_TICK if delta > 0 else -MAX_PWM_DELTA_PER_TICK
    return max(0.0, min(100.0, applied + delta))

def motor_ramp_tick():
    global _motor_applied_duty_l, _motor_applied_duty_r
    global _last_motor_fwd, _brake_active
    global _dwell_active, _dwell_start_time

    # ── Non-blocking brake state machine ──
    if _brake_active:
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, _brake_start_time)

        # All direction pins HIGH for magnetic brake short
        mot_in1.value(1); mot_in2.value(1)
        mot_in3.value(1); mot_in4.value(1)

        if elapsed < 100:
            progress = elapsed / 100.0
            _set_motor_pwm_l(_brake_start_duty_l + (100.0 - _brake_start_duty_l) * progress)
            _set_motor_pwm_r(_brake_start_duty_r + (100.0 - _brake_start_duty_r) * progress)
        elif elapsed < 150:
            _set_motor_pwm(100)
        else:
            _set_motor_pwm(0)
            mot_in1.value(0); mot_in2.value(0)
            mot_in3.value(0); mot_in4.value(0)
            _motor_applied_duty_l = 0.0
            _motor_applied_duty_r = 0.0
            _last_motor_fwd = None
            _brake_active = False

        return  # Skip normal motor ramping while braking

    # ── Non-blocking direction-change dwell ──
    if _dwell_active:
        if time.ticks_diff(time.ticks_ms(), _dwell_start_time) < DIRECTION_CHANGE_DWELL_MS:
            return  # Still dwelling — skip motor update
        _dwell_active = False
        _last_motor_fwd = _motor_forward

    if _motor_forward != _last_motor_fwd and _last_motor_fwd is not None:
        _set_motor_pwm(0)
        _motor_applied_duty_l = 0.0
        _motor_applied_duty_r = 0.0
        _dwell_active = True
        _dwell_start_time = time.ticks_ms()
        return  # Start dwell, resume on next tick

    _set_motor_pins(_motor_forward)
    _last_motor_fwd = _motor_forward
    _motor_applied_duty_l = _ramp_one(_motor_applied_duty_l, float(_motor_target_duty_l))
    _motor_applied_duty_r = _ramp_one(_motor_applied_duty_r, float(_motor_target_duty_r))
    _set_motor_pwm_l(_motor_applied_duty_l)
    _set_motor_pwm_r(_motor_applied_duty_r)

def motor_brake():
    """Triggers the non-blocking magnetic brake sequence.
    Actual braking is handled by motor_ramp_tick() state machine."""
    global _brake_active, _brake_start_time, _brake_start_duty_l, _brake_start_duty_r
    global _motor_target_duty_l, _motor_target_duty_r
    _brake_active = True
    _brake_start_time = time.ticks_ms()
    _brake_start_duty_l = _motor_applied_duty_l
    _brake_start_duty_r = _motor_applied_duty_r
    _motor_target_duty_l = 0
    _motor_target_duty_r = 0

def motor_stop():
    global _motor_applied_duty_l, _motor_applied_duty_r
    global _motor_target_duty_l, _motor_target_duty_r, _last_motor_fwd
    _set_motor_pwm(0)
    mot_in1.value(0); mot_in2.value(0)
    mot_in3.value(0); mot_in4.value(0)
    _motor_applied_duty_l = 0.0; _motor_applied_duty_r = 0.0
    _motor_target_duty_l = 0; _motor_target_duty_r = 0
    _last_motor_fwd = None

motor_stop()
print("✅ L298N motors: L(ENA=GP10,IN1=GP17,IN2=GP12) R(ENB=GP16,IN3=GP13,IN4=GP14)")

# =====================================================
# UART COMMAND PARSER
# MC:<speed>,<steer_pw>\n  — Forward + steering
# MR:<speed>,<steer_pw>\n  — Reverse + steering
# ML:<left>,<right>\n      — Independent L/R PWM (forward)
# MB\n / MS\n              — Brake / Stop
# PT:<pan>,<tilt>\n        — Pan-tilt control (pan: 0-180°, tilt: 45-135°)
# PC\n                     — Pan-tilt center (90°, 90°)
# ER\n                     — Encoder reset
# =====================================================
_uart_buf = ""

def _check_uart_commands():
    global _uart_buf, _pan_angle, _tilt_angle
    global _motor_target_duty_l, _motor_target_duty_r, _motor_forward, _last_cmd_time
    while uart.any():
        ch = uart.read(1)
        if ch is None:
            break
        ch = ch.decode('ascii', 'ignore')
        if ch == '\n':
            line = _uart_buf.strip()
            _uart_buf = ""
            if line.startswith("MC:"):
                try:
                    parts = line[3:].split(",")
                    spd = _servo_clamp(int(parts[0]), 0, 100)
                    _motor_target_duty_l = spd
                    _motor_target_duty_r = spd
                    _motor_forward = True
                    set_steering_pw(int(parts[1]))
                    _last_cmd_time = time.ticks_ms()
                except Exception:
                    pass
            elif line.startswith("MR:"):
                try:
                    parts = line[3:].split(",")
                    spd = _servo_clamp(int(parts[0]), 0, 100)
                    _motor_target_duty_l = spd
                    _motor_target_duty_r = spd
                    _motor_forward = False
                    set_steering_pw(int(parts[1]))
                    _last_cmd_time = time.ticks_ms()
                except Exception:
                    pass
            elif line.startswith("ML:"):
                try:
                    parts = line[3:].split(",")
                    new_l = _servo_clamp(int(parts[0]), 0, 100)
                    new_r = _servo_clamp(int(parts[1]), 0, 100)
                    _motor_target_duty_l = new_l
                    _motor_target_duty_r = new_r
                    if len(parts) >= 3:
                        set_steering_pw(int(parts[2]))
                    if len(parts) >= 4:
                        _motor_forward = (parts[3] == 'F')
                    else:
                        _motor_forward = True
                    _last_cmd_time = time.ticks_ms()
                except Exception:
                    pass
            elif line == "MB":
                motor_brake()
                _last_cmd_time = time.ticks_ms()
            elif line == "MS":
                motor_stop()
                _last_cmd_time = time.ticks_ms()
            elif line.startswith("PT:"):
                try:
                    parts = line[3:].split(",")
                    _pan_angle  = _servo_clamp(int(parts[0]), PAN_MIN, PAN_MAX)
                    _tilt_angle = _servo_clamp(int(parts[1]), TILT_MIN, TILT_MAX)
                    _apply_pan_tilt()
                except Exception:
                    pass
            elif line == "PC":
                _pan_angle = PAN_CENTER; _tilt_angle = TILT_CENTER
                _apply_pan_tilt()
            elif line == "ER":
                enc_left.reset_position()
                enc_right.reset_position()
        else:
            _uart_buf += ch
            if len(_uart_buf) > 64:
                _uart_buf = ""

# =====================================================
# I2C SETUP (MPU6500 + VL53L0X + ADS1115 + QMC5883L)
# Confirmed: scl=GP9, sda=GP8, freq=400kHz
# =====================================================
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

time.sleep(1)
devices = i2c.scan()
print(f"✅ I2C devices: {[hex(d) for d in devices]}")
print(f"   Expected: 0x0d(QMC5883L), 0x29(VL53L0X), 0x48(ADS1115), 0x68(MPU6500)\n")

# ── MPU6500 (IMU) — 0x68 ──
MPU_ADDR = 0x68
mpu_ok = False

def mpu_write(reg, data):
    try:
        i2c.writeto_mem(MPU_ADDR, reg, bytes([data]))
        return True
    except OSError:
        return False

def mpu_read(reg, length):
    try:
        return i2c.readfrom_mem(MPU_ADDR, reg, length)
    except OSError:
        return None

if mpu_write(0x6B, 0x00):
    time.sleep(0.1)
    mpu_ok = True
    print("✅ MPU6500 initialized")
else:
    print("⚠️  MPU6500 failed to initialize")

# ── VL53L0X (laser) — 0x29 ──
VL53_ADDR = 0x29
vl53_ok = False

def vl53_init():
    try:
        i2c.writeto_mem(VL53_ADDR, 0x00, b'\x01')
        time.sleep_ms(50)
        return True
    except OSError:
        return False

def vl53_single_measure():
    try:
        i2c.writeto_mem(VL53_ADDR, 0x00, b'\x01')
        time.sleep_ms(35)
        data = i2c.readfrom_mem(VL53_ADDR, 0x14, 12)
        d = (data[10] << 8) | data[11]
        return d if 30 <= d <= 2000 else -1
    except OSError:
        return -1

vl53_ok = vl53_init()
print(f"{'✅' if vl53_ok else '⚠️ '} VL53L0X {'initialized' if vl53_ok else 'not responding'}")

# ── QMC5883L (magnetometer) — 0x0D ──
QMC_ADDR = 0x0D
qmc_ok = False

def qmc_init():
    try:
        i2c.writeto_mem(QMC_ADDR, 0x0B, b'\x01')
        time.sleep_ms(10)
        i2c.writeto_mem(QMC_ADDR, 0x09, b'\x1D')
        time.sleep_ms(100)
        return True
    except OSError:
        return False

def qmc_read():
    try:
        data = i2c.readfrom_mem(QMC_ADDR, 0x00, 6)
        x, y, z = struct.unpack('<hhh', data)
        return round(x / 3000.0, 4), round(y / 3000.0, 4), round(z / 3000.0, 4)
    except OSError:
        return None, None, None

qmc_ok = qmc_init()
print(f"{'✅' if qmc_ok else '⚠️ '} QMC5883L {'initialized' if qmc_ok else 'not found'}")

# ── ADS1115 (4-channel ADC) — 0x48 ──
ADS_ADDR = 0x48
ads_ok = False

def ads_init():
    try:
        i2c.readfrom_mem(ADS_ADDR, 0x01, 2)
        return True
    except OSError:
        return False

def ads_read_channel(channel):
    if channel < 0 or channel > 3:
        return -1
    try:
        mux = (4 + channel) << 4
        cfg_hi = 0x80 | mux | 0x02 | 0x01
        i2c.writeto_mem(ADS_ADDR, 0x01, bytes([cfg_hi, 0x83]))
        time.sleep_ms(10)
        data = i2c.readfrom_mem(ADS_ADDR, 0x00, 2)
        raw = struct.unpack('>h', data)[0]
        mv = raw * 0.125
        return round(max(0.0, mv), 1)
    except OSError:
        return -1

ads_ok = ads_init()
print(f"{'✅' if ads_ok else '⚠️ '} ADS1115 {'initialized' if ads_ok else 'not found'}")


# #####################################################
# DUAL-CORE ARCHITECTURE
# #####################################################
#
# Data flow: Core 1 writes RPM → Core 0 reads RPM + sensors → UART TX
# UART TX moved to Core 0 (main thread) for reliability —
# MicroPython _thread UART writes are unreliable on RP2040.

# ── Shared sensor data (Core 0 → Core 1) ──
_sensor_data = {
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro":  {"x": 0.0, "y": 0.0, "z": 0.0},
    "temp_c": 0.0,
    "mag":   {"x": 0.0, "y": 0.0, "z": 0.0},
    "laser_mm": -1,
    "adc":   {"A0": -1, "A1": -1, "A2": -1, "A3": -1},
}

_core0_errors = 0
_c1_errors = 0
_core1_started = False


# =====================================================
# CORE 0 — Sensor Loop (main thread, 50 Hz)
# =====================================================
def core0_sensor_loop():
    """Read all I2C sensors + UART TX JSON telemetry from main thread."""
    global _sensor_data, _core0_errors

    print("🟢 [Core 0] Sensor + UART TX loop started")

    tx_frame = 0

    while True:
        try:
            t0 = time.ticks_ms()

            # ── MPU6500: Accel + Gyro + Temp (14 bytes burst) ──
            accel = {"x": 0.0, "y": 0.0, "z": 0.0}
            gyro  = {"x": 0.0, "y": 0.0, "z": 0.0}
            temp_c = 0.0
            if mpu_ok:
                raw = mpu_read(0x3B, 14)
                if raw is not None:
                    ax, ay, az, t, gx, gy, gz = struct.unpack(">hhhhhhh", raw)
                    accel = {"x": round(ax / 16384.0, 3),
                             "y": round(ay / 16384.0, 3),
                             "z": round(az / 16384.0, 3)}
                    gyro  = {"x": round(gx / 131.0, 2),
                             "y": round(gy / 131.0, 2),
                             "z": round(gz / 131.0, 2)}
                    temp_c = round((t / 333.87) + 21.0, 1)
                else:
                    _core0_errors += 1

            # ── VL53L0X: Laser distance ──
            laser_mm = vl53_single_measure() if vl53_ok else -1

            # ── QMC5883L: Magnetometer ──
            if qmc_ok:
                mx, my, mz = qmc_read()
                mag = {"x": mx, "y": my, "z": mz} if mx is not None else {"x": 0.0, "y": 0.0, "z": 0.0}
            else:
                mag = {"x": 0.0, "y": 0.0, "z": 0.0}

            # ── ADS1115: 4-channel ADC ──
            if ads_ok:
                adc = {"A0": ads_read_channel(0), "A1": ads_read_channel(1),
                       "A2": ads_read_channel(2), "A3": ads_read_channel(3)}
            else:
                adc = {"A0": -1, "A1": -1, "A2": -1, "A3": -1}

            # ── Atomic update: replace dict reference ──
            _sensor_data = {
                "accel": accel, "gyro": gyro, "temp_c": temp_c,
                "mag": mag, "laser_mm": laser_mm, "adc": adc,
            }

            # ── LED heartbeat ──
            led.on(); time.sleep_ms(2); led.off()

            # ── UART RX command parsing (on Core 0 for thread safety) ──
            _check_uart_commands()

            # ── UART TX JSON telemetry (on Core 0 for thread safety) ──
            # Compact keys + arrays to keep packets under ~220 bytes
            # (115200 baud = 230 byte budget per 20ms frame)
            packet = {
                "t": time.ticks_ms(),
                "f": tx_frame,
                "a": [accel["x"], accel["y"], accel["z"]],
                "g": [gyro["x"], gyro["y"], gyro["z"]],
                "tc": temp_c,
                "m": [mag["x"], mag["y"], mag["z"]],
                "l": laser_mm,
                "d": [adc["A0"], adc["A1"], adc["A2"], adc["A3"]],
                "e": [_enc_pos_left, _enc_pos_right],
                "r": [round(_rpm_filtered_left, 1), round(_rpm_filtered_right, 1),
                       round(_rpm_left, 1), round(_rpm_right, 1)],
                "mt": [round(_motor_applied_duty_l, 1), round(_motor_applied_duty_r, 1)],
                "er": _c1_errors + _core0_errors,
            }
            try:
                line = json.dumps(packet, separators=(',', ':'))
            except TypeError:
                line = json.dumps(packet)
            uart.write(line)
            uart.write("\n")
            tx_frame += 1
            gc.collect()

            # ── Maintain loop period ──
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < 20:
                time.sleep_ms(20 - elapsed)

        except Exception as e:
            _core0_errors += 1
            print(f"❌ [Core 0] {e}")
            time.sleep_ms(20)


# =====================================================
# CORE 1 — Fast Loop (1 kHz tick, dividers for subtasks)
# =====================================================
def core1_fast_loop():
    """Core 1: motor control @ 100 Hz, RPM from PIO encoder counts."""
    global _rpm_left, _rpm_right, _rpm_filtered_left, _rpm_filtered_right
    global _enc_pos_left, _enc_pos_right
    global _core1_started, _c1_errors

    _core1_started = True
    print("🔴 [Core 1] Fast loop started — motor+RPM @100Hz")

    prev_rpm_time = time.ticks_ms()
    prev_raw_l = enc_left.get_raw_count()
    prev_raw_r = enc_right.get_raw_count()

    while True:
        try:
            t0 = time.ticks_ms()

            # UART RX — now handled by Core 0 for thread safety

            # Motor watchdog
            if (_motor_target_duty_l > 0 or _motor_target_duty_r > 0) and time.ticks_diff(t0, _last_cmd_time) > WATCHDOG_TIMEOUT_MS:
                motor_stop()

            # Motor ramp
            motor_ramp_tick()

            # RPM from PIO encoder raw count deltas
            dt_ms = time.ticks_diff(t0, prev_rpm_time)
            if dt_ms > 0:
                prev_rpm_time = t0
                raw_l = enc_left.get_raw_count()
                raw_r = enc_right.get_raw_count()
                dl = raw_l - prev_raw_l
                dr = raw_r - prev_raw_r
                prev_raw_l = raw_l
                prev_raw_r = raw_r

                _enc_pos_left  = enc_left.get_position()
                _enc_pos_right = enc_right.get_position()

                _rpm_left  = (dl / ENCODER_PPR) * (60000.0 / dt_ms)
                _rpm_right = (dr / ENCODER_PPR) * (60000.0 / dt_ms)
                _rpm_filtered_left  = ALPHA_RPM * _rpm_left  + (1.0 - ALPHA_RPM) * _rpm_filtered_left
                _rpm_filtered_right = ALPHA_RPM * _rpm_right + (1.0 - ALPHA_RPM) * _rpm_filtered_right

            # ── Maintain 100 Hz (10 ms period) ──
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            remaining = 10 - elapsed
            if remaining > 0:
                time.sleep_ms(remaining)

        except Exception as e:
            _c1_errors += 1
            time.sleep_us(1000)


# =====================================================
# LAUNCH DUAL-CORE
# =====================================================
print("\n" + "="*60)
print("  🚀 LAUNCHING DUAL-CORE ARCHITECTURE...")
print("="*60 + "\n")

# UART TX test — verify wiring before dual-core launch
for _i in range(5):
    uart.write(b'{"test":"pico_alive","n":' + str(_i).encode() + b'}\n')
    time.sleep_ms(50)
print("📡 UART test packets sent on GP0 (5x)")

# Core 1: fast loop (motors, PIO encoders)
_thread.start_new_thread(core1_fast_loop, ())
time.sleep_ms(200)

if _core1_started:
    print("✅ Core 1 running — Motor+RPM @100Hz")
else:
    print("⚠️  Core 1 may not have started yet")

# Core 0: sensor + UART TX loop (main thread)
print("✅ Core 0 running — I2C Sensors + UART TX")
print("\n" + "="*60)
print("  📡 DUAL-CORE SENSOR BRIDGE ACTIVE")
print("="*60 + "\n")

core0_sensor_loop()
