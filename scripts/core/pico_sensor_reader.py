"""
pico_sensor_reader.py — Pico UART Sensor Bridge Reader
Integrates Pico sensor data into autonomous driving controller
"""

import serial
import json
import threading
import time
from collections import deque
from dataclasses import dataclass


@dataclass
class SensorPacket:
    """Parsed sensor data from Pico."""
    timestamp_ms: int
    frame: int

    # MPU6500 (accel in g, gyro in deg/s)
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    temp_c: float

    # HMC5883L magnetometer (Gauss)
    mag_x: float
    mag_y: float
    mag_z: float

    # VL53L0X laser (mm)
    laser_mm: int

    # ADS1115 ADC (mV)
    adc_a0: float  # Channel A0 (e.g., battery voltage)
    adc_a1: float  # Channel A1 (e.g., current sense)
    adc_a2: float  # Channel A2 (spare)
    adc_a3: float  # Channel A3 (spare)

    # Wheel encoders (cumulative step counts from Pico quadrature)
    enc_left_steps: int   # Left encoder cumulative steps (4× counting)
    enc_right_steps: int  # Right encoder cumulative steps (4× counting)

    # RPM (calculated on Pico from encoder deltas each 20ms frame)
    rpm_left: float       # Left wheel filtered RPM
    rpm_right: float      # Right wheel filtered RPM
    rpm_left_raw: float   # Left wheel instantaneous RPM (unfiltered)
    rpm_right_raw: float  # Right wheel instantaneous RPM (unfiltered)

    # Motor applied duties (Pico-side, after ramp)
    mot_duty_left: float   # Left motor applied duty %
    mot_duty_right: float  # Right motor applied duty %

    # Error counter
    errors: int


class PicoSensorReader:
    """
    Reads JSON sensor packets from Pico over UART.
    Provides thread-safe access to latest sensor data.
    """

    # Pan/tilt servo limits (Pico GP2=pan, GP3=tilt)
    PAN_CENTER  = 90;  TILT_CENTER = 90
    PAN_MIN = 60;  PAN_MAX = 120
    TILT_MIN = 60; TILT_MAX = 120

    # Ackermann steering servo bounds (Pico GP15).
    # The Pi owns the actual chassis calibration; the bridge keeps only a
    # broad absolute clamp so calibration can change without reflashing.
    STEER_CENTER_PW = 1440   # µs — fallback straight-ahead pulse
    STEER_MIN_PW = 500       # µs — broad low-level servo guard
    STEER_MAX_PW = 2500      # µs — broad low-level servo guard
    # Backward-compatible aliases for older callers.
    STEER_LEFT_PW = STEER_MIN_PW
    STEER_RIGHT_PW = STEER_MAX_PW

    def __init__(self, port='/dev/ttyS0', baudrate=115200, buffer_size=10):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._last_packet = None
        self._last_packet_time = 0.0  # monotonic time of last received packet
        self._packet_buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()  # serialise UART writes
        self._running = False
        self._error_count = 0
        self._packet_count = 0
        self._read_thread = None
        
        # Fault detection metrics
        self._consecutive_errors = 0  # Count of parse errors in a row
        self._last_error_time = 0.0  # When last error occurred
        self._error_rate_window = deque(maxlen=100)  # Recent 100 packets (error=True/False)

        self._connect()

    def _connect(self):
        """Attempt to establish serial connection."""
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✅ Pico sensor reader connected: {self.port} @ {self.baudrate} baud")
            self._running = True

            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

        except FileNotFoundError:
            print(f"❌ Serial port not found: {self.port}")
            print("   Check: /dev/ttyAMA0, /dev/ttyS0, or /dev/serial0")
            self._running = False
        except PermissionError:
            print(f"❌ Permission denied on {self.port}")
            print("   Fix: sudo usermod -a -G dialout $USER && logout/login")
            self._running = False
        except Exception as e:
            print(f"❌ Failed to open serial: {e}")
            self._running = False

    @staticmethod
    def _parse_packet(raw):
        """Parse a JSON dict into a SensorPacket, supporting both compact and legacy keys."""
        # Accelerometer: compact "a":[x,y,z] or legacy "accel":{"x":..}
        a = raw.get('a')
        if isinstance(a, list):
            ax, ay, az = a[0], a[1], a[2]
        else:
            accel = raw.get('accel', {})
            ax, ay, az = accel.get('x', 0.0), accel.get('y', 0.0), accel.get('z', 0.0)

        # Gyroscope: compact "g":[x,y,z] or legacy "gyro":{"x":..}
        g = raw.get('g')
        if isinstance(g, list):
            gx, gy, gz = g[0], g[1], g[2]
        else:
            gyro = raw.get('gyro', {})
            gx, gy, gz = gyro.get('x', 0.0), gyro.get('y', 0.0), gyro.get('z', 0.0)

        # Magnetometer: compact "m":[x,y,z] or legacy "mag":{"x":..}
        m = raw.get('m')
        if isinstance(m, list):
            mx, my, mz = m[0], m[1], m[2]
        else:
            mag = raw.get('mag', {})
            mx, my, mz = mag.get('x', 0.0), mag.get('y', 0.0), mag.get('z', 0.0)

        # ADC: compact "d":[A0,A1,A2,A3] or legacy "adc":{"A0":..}
        d = raw.get('d')
        if isinstance(d, list):
            da0, da1, da2, da3 = d[0], d[1], d[2], d[3]
        else:
            adc = raw.get('adc', {})
            da0, da1, da2, da3 = adc.get('A0', -1), adc.get('A1', -1), adc.get('A2', -1), adc.get('A3', -1)

        # Encoders: compact "e":[left,right] or legacy "enc":{"l":..}
        e = raw.get('e')
        if isinstance(e, list):
            el, er = e[0], e[1]
        else:
            enc = raw.get('enc', {})
            el, er = enc.get('l', 0), enc.get('r', 0)

        # RPM: compact "r":[l,r,l_raw,r_raw] or legacy "rpm":{"l":..}
        r = raw.get('r')
        if isinstance(r, list):
            rl, rr = r[0], r[1]
            rl_raw = r[2] if len(r) > 2 else 0.0
            rr_raw = r[3] if len(r) > 3 else 0.0
        else:
            rpm = raw.get('rpm', {})
            rl, rr = rpm.get('l', 0.0), rpm.get('r', 0.0)
            rl_raw, rr_raw = rpm.get('l_raw', 0.0), rpm.get('r_raw', 0.0)

        # Motor duty: compact "mt":[l,r] or legacy "mot":{"l":..}
        mt = raw.get('mt')
        if isinstance(mt, list):
            ml, mr = mt[0], mt[1]
        else:
            mot = raw.get('mot', {})
            ml, mr = mot.get('l', 0.0), mot.get('r', 0.0)

        return SensorPacket(
            timestamp_ms=raw.get('t', raw.get('ts', 0)),
            frame=raw.get('f', raw.get('frame', 0)),
            accel_x=ax, accel_y=ay, accel_z=az,
            gyro_x=gx, gyro_y=gy, gyro_z=gz,
            temp_c=raw.get('tc', raw.get('temp_c', 0.0)),
            mag_x=mx, mag_y=my, mag_z=mz,
            laser_mm=raw.get('l', raw.get('laser_mm', -1)),
            adc_a0=da0, adc_a1=da1, adc_a2=da2, adc_a3=da3,
            enc_left_steps=el, enc_right_steps=er,
            rpm_left=rl, rpm_right=rr,
            rpm_left_raw=rl_raw, rpm_right_raw=rr_raw,
            mot_duty_left=ml, mot_duty_right=mr,
            errors=raw.get('er', raw.get('errors', 0)),
        )

    def _read_loop(self):
        """Background thread: continuously read and parse JSON packets."""
        while self._running and self._serial:
            try:
                line = self._serial.readline().decode().strip()

                if not line:
                    continue

                # Parse JSON
                if line.startswith('{'):
                    try:
                        raw = json.loads(line)

                        # Handle Pico startup test packets
                        if 'test' in raw:
                            print(f"📡 Pico test packet received: {raw}")
                            continue

                        packet = self._parse_packet(raw)

                        with self._lock:
                            self._last_packet = packet
                            self._last_packet_time = time.monotonic()
                            self._packet_buffer.append(packet)
                            self._packet_count += 1
                            self._consecutive_errors = 0  # Reset on success
                            self._error_rate_window.append(False)  # No error this packet

                    except (json.JSONDecodeError, KeyError, TypeError, IndexError) as e:
                        self._error_count += 1
                        self._consecutive_errors += 1
                        self._last_error_time = time.time()
                        self._error_rate_window.append(True)  # Error this packet
                        print(f"⚠️  Parse error: {e} | Data: {line[:80]}")

            except Exception as e:
                if not self._running:
                    break
                self._error_count += 1
                self._consecutive_errors += 1
                self._last_error_time = time.time()
                print(f"⚠️  Read error: {e}")
                time.sleep(0.01)

    def get_latest(self):
        with self._lock:
            return self._last_packet

    def get_buffer(self):
        with self._lock:
            return list(self._packet_buffer)

    def is_connected(self):
        with self._lock:
            return self._running and self._last_packet is not None

    def seconds_since_last_packet(self):
        """Return seconds since the last valid packet, or -1 if none ever received."""
        with self._lock:
            if self._last_packet_time == 0.0:
                return -1
            return time.monotonic() - self._last_packet_time

    def is_fresh(self, max_age_s=3.0):
        """Return True if a packet was received within max_age_s seconds."""
        age = self.seconds_since_last_packet()
        return age >= 0 and age <= max_age_s

    def get_stats(self):
        with self._lock:
            return {
                "packets_received": self._packet_count,
                "errors": self._error_count,
                "connected": self._running,
                "consecutive_errors": self._consecutive_errors,
                "error_rate_recent": self._get_error_rate(),
            }
    
    def _get_error_rate(self):
        """Return recent error rate (0.0-1.0) from last 100 read attempts."""
        if not self._error_rate_window:
            return 0.0
        errors = sum(self._error_rate_window)
        return errors / len(self._error_rate_window)
    
    def get_connection_health(self):
        """
        Return True if connection is healthy, False if degraded.
        Considers packet freshness, error rate, and consecutive errors.
        """
        with self._lock:
            if not self._running or self._last_packet is None:
                return False
            
            age_s = time.monotonic() - self._last_packet_time
            
            # Connection is unhealthy if:
            # - No packet received in >5 seconds, OR
            # - 3+ consecutive parse errors, OR
            # - Error rate > 20% in recent window
            if age_s > 5.0:
                return False
            if self._consecutive_errors >= 3:
                return False
            if self._get_error_rate() > 0.20:
                return False
            
            return True

    # ── Pan/tilt UART commands ─────────────────────────────────

    def send_pan_tilt(self, pan, tilt):
        """Send PT:<pan>,<tilt> command to Pico to move pan/tilt servos."""
        if not self._serial or not self._running:
            return
        pan = max(self.PAN_MIN, min(self.PAN_MAX, int(pan)))
        tilt = max(self.TILT_MIN, min(self.TILT_MAX, int(tilt)))
        with self._write_lock:
            try:
                self._serial.write(f"PT:{pan},{tilt}\n".encode())
            except Exception:
                pass

    def send_center(self):
        """Send PC command to centre both pan and tilt servos."""
        if not self._serial or not self._running:
            return
        with self._write_lock:
            try:
                self._serial.write(b"PC\n")
            except Exception:
                pass

    # ── Motor / steering UART commands ─────────────────────────

    def send_motor_command(self, speed, steer_pw):
        """Send MC:<speed>,<steer_pw> to drive forward with steering.
        speed: 0-100 (duty %), steer_pw: absolute pulse width in µs."""
        if not self._serial or not self._running:
            return
        speed = max(0, min(100, int(speed)))
        steer_pw = max(self.STEER_MIN_PW, min(self.STEER_MAX_PW, int(steer_pw)))
        with self._write_lock:
            try:
                self._serial.write(f"MC:{speed},{steer_pw}\n".encode())
            except Exception:
                pass

    def send_reverse_command(self, speed, steer_pw):
        """Send MR:<speed>,<steer_pw> to drive in reverse with steering.
        speed: 0-100 (duty %), steer_pw: absolute pulse width in µs."""
        if not self._serial or not self._running:
            return
        speed = max(0, min(100, int(speed)))
        steer_pw = max(self.STEER_MIN_PW, min(self.STEER_MAX_PW, int(steer_pw)))
        with self._write_lock:
            try:
                self._serial.write(f"MR:{speed},{steer_pw}\n".encode())
            except Exception:
                pass

    def send_brake(self):
        """Send MB command: H-bridge magnetic brake."""
        if not self._serial or not self._running:
            return
        with self._write_lock:
            try:
                self._serial.write(b"MB\n")
            except Exception:
                pass

    def send_stop(self):
        """Send MS command: coast stop (all pins LOW)."""
        if not self._serial or not self._running:
            return
        with self._write_lock:
            try:
                self._serial.write(b"MS\n")
            except Exception:
                pass

    def send_encoder_reset(self):
        """Send ER command: reset Pico encoder step counts to zero."""
        if not self._serial or not self._running:
            return
        with self._write_lock:
            try:
                self._serial.write(b"ER\n")
            except Exception:
                pass

    def send_lr_pwm(self, left_pwm, right_pwm, steer_pw=None, forward=True):
        """Send ML:<left>,<right>[,<steer_pw>,<F|R>] for independent L/R motor PWM.
        left_pwm, right_pwm: 0-100 (duty %).
        steer_pw: optional steering servo pulse width in µs.
        forward: direction flag (True=forward, False=reverse)."""
        if not self._serial or not self._running:
            return
        left_pwm = max(0, min(100, int(left_pwm)))
        right_pwm = max(0, min(100, int(right_pwm)))
        cmd = f"ML:{left_pwm},{right_pwm}"
        if steer_pw is not None:
            steer_pw = max(self.STEER_MIN_PW, min(self.STEER_MAX_PW, int(steer_pw)))
            direction = 'F' if forward else 'R'
            cmd += f",{steer_pw},{direction}"
        with self._write_lock:
            try:
                self._serial.write(f"{cmd}\n".encode())
            except Exception:
                pass

    def send_steering_pw(self, steer_pw):
        """Move the steering servo without applying drive power."""
        self.send_lr_pwm(0, 0, steer_pw=steer_pw, forward=True)

    def close(self):
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self._serial:
            self._serial.close()
        print("\n🛑 Pico sensor reader closed")


# ============================================================
# CONVENIENCE FUNCTIONS
# ============================================================

_global_reader = None

def init_pico_reader(port='/dev/ttyS0'):
    global _global_reader
    if _global_reader is None:
        _global_reader = PicoSensorReader(port)
    return _global_reader

def get_sensor_packet():
    if _global_reader:
        return _global_reader.get_latest()
    return None

def get_gyro_z():
    packet = get_sensor_packet()
    return packet.gyro_z if packet else 0.0

def get_accel_xyz():
    packet = get_sensor_packet()
    if packet:
        return packet.accel_x, packet.accel_y, packet.accel_z
    return 0.0, 0.0, 0.0

def get_laser_distance_mm():
    packet = get_sensor_packet()
    return packet.laser_mm if packet else -1

def get_laser_distance_cm():
    mm = get_laser_distance_mm()
    return round(mm / 10.0, 1) if mm > 0 else -1

def get_magnetometer():
    """Return (mag_x, mag_y, mag_z) in Gauss from QMC5883L."""
    packet = get_sensor_packet()
    if packet:
        return packet.mag_x, packet.mag_y, packet.mag_z
    return 0.0, 0.0, 0.0


def send_pan_tilt(pan, tilt):
    """Send pan/tilt servo command to Pico via UART.
    pan/tilt in degrees (60-120, center=90)."""
    if _global_reader:
        _global_reader.send_pan_tilt(pan, tilt)


def send_center():
    """Centre both pan/tilt servos via Pico UART."""
    if _global_reader:
        _global_reader.send_center()


def send_motor_command(speed, steer_pw):
    """Send forward motor + steering command to Pico.
    speed: 0-100 (duty %), steer_pw: absolute pulse width in µs."""
    if _global_reader:
        _global_reader.send_motor_command(speed, steer_pw)

def send_reverse_command(speed, steer_pw):
    """Send reverse motor + steering command to Pico.
    speed: 0-100 (duty %), steer_pw: absolute pulse width in µs."""
    if _global_reader:
        _global_reader.send_reverse_command(speed, steer_pw)

def send_brake():
    """Send magnetic brake command to Pico."""
    if _global_reader:
        _global_reader.send_brake()

def send_stop():
    """Send coast stop command to Pico."""
    if _global_reader:
        _global_reader.send_stop()

def send_encoder_reset():
    """Reset Pico encoder step counts to zero."""
    if _global_reader:
        _global_reader.send_encoder_reset()

def send_lr_pwm(left_pwm, right_pwm, steer_pw=None, forward=True):
    """Send independent L/R motor PWM. 0-100 duty %. Optional steering PW + direction."""
    if _global_reader:
        _global_reader.send_lr_pwm(left_pwm, right_pwm, steer_pw, forward)


def send_steering_pw(steer_pw):
    """Move the steering servo without applying drive power."""
    if _global_reader:
        _global_reader.send_steering_pw(steer_pw)


class PicoEncoderProxy:
    """Drop-in replacement for LgpioEncoder that reads steps from Pico UART.

    Exposes a .steps property matching LgpioEncoder's interface so that
    motor.py rpm_pid_tick() and main.py encoder_and_power_thread() work
    unchanged — they just read .steps and compute RPM from deltas.
    """

    def __init__(self, reader, side='left'):
        self._reader = reader
        self._side = side

    @property
    def steps(self):
        pkt = self._reader.get_latest()
        if pkt is None:
            return 0
        return pkt.enc_left_steps if self._side == 'left' else pkt.enc_right_steps

    def cancel(self):
        """No-op: cleanup not needed for UART proxy."""
        pass


def get_encoder_proxy_left():
    """Return a PicoEncoderProxy for the left wheel (reads from Pico UART)."""
    if _global_reader:
        return PicoEncoderProxy(_global_reader, 'left')
    return None

def get_encoder_proxy_right():
    """Return a PicoEncoderProxy for the right wheel (reads from Pico UART)."""
    if _global_reader:
        return PicoEncoderProxy(_global_reader, 'right')
    return None


def is_pico_fresh():
    """Return True if the Pico sensor data was received recently (within 3 s)."""
    global _global_reader
    return bool(_global_reader and _global_reader.is_fresh())

def get_diagnostics():
    """Return connection diagnostics dict for debugging UI."""
    if _global_reader is None:
        return {'connected': False, 'fresh': False, 'frames': 0, 'errors': 0,
                'packets_received': 0, 'laser_mm': -1, 'age_s': -1}
    packet = _global_reader.get_latest()
    stats = _global_reader.get_stats()
    age = _global_reader.seconds_since_last_packet()
    return {
        'connected': _global_reader.is_connected(),
        'fresh': _global_reader.is_fresh(),
        'frames': packet.frame if packet else 0,
        'errors': packet.errors if packet else 0,
        'packets_received': stats.get('packets_received', 0),
        'laser_mm': packet.laser_mm if packet else -1,
        'age_s': round(age, 2) if age >= 0 else -1,
    }

def get_adc_voltage_mv(channel='A0'):
    packet = get_sensor_packet()
    if not packet:
        return -1

    channel = channel.upper()
    if channel == 'A0':
        return packet.adc_a0
    elif channel == 'A1':
        return packet.adc_a1
    elif channel == 'A2':
        return packet.adc_a2
    elif channel == 'A3':
        return packet.adc_a3
    else:
        return -1

def get_battery_voltage(divider_ratio=5.0):
    adc_mv = get_adc_voltage_mv('A0')
    if adc_mv < 0:
        return -1
    return (adc_mv / 1000.0) * divider_ratio

def get_current_sense(shunt_ohms=0.1):
    adc_mv = get_adc_voltage_mv('A1')
    if adc_mv < 0:
        return -1
    adc_v = adc_mv / 1000.0
    return adc_v / shunt_ohms


def get_pico_rpm():
    """Return (rpm_left, rpm_right) as calculated by the Pico (filtered)."""
    packet = get_sensor_packet()
    if packet:
        return packet.rpm_left, packet.rpm_right
    return 0.0, 0.0

def get_pico_motor_duty():
    """Return (duty_left, duty_right) applied motor duty % from Pico."""
    packet = get_sensor_packet()
    if packet:
        return packet.mot_duty_left, packet.mot_duty_right
    return 0.0, 0.0

def get_pico_rpm_raw():
    """Return (rpm_left_raw, rpm_right_raw) unfiltered instantaneous RPM from Pico."""
    packet = get_sensor_packet()
    if packet:
        return packet.rpm_left_raw, packet.rpm_right_raw
    return 0.0, 0.0

def get_pico_rpm_avg():
    """Return average filtered RPM of both wheels (from Pico)."""
    rpm_l, rpm_r = get_pico_rpm()
    return (rpm_l + rpm_r) / 2.0


if __name__ == "__main__":
    # Test standalone with standard scrolling output
    reader = PicoSensorReader()

    print("Waiting for packets...\n")
    time.sleep(1)

    try:
        while True:
            packet = reader.get_latest()
            
            if packet:
                # Calculate human-readable values
                volts = (packet.adc_a0 / 1000.0) * 5.0
                amps = (packet.adc_a1 / 1000.0) / 0.1
                
                # Print a compact, single-line scrolling log
                print(f"[Frame {packet.frame:05d}] V:{volts:5.2f}V | I:{amps:5.2f}A | "
                      f"Laser:{packet.laser_mm:4}mm | "
                      f"RPM L:{packet.rpm_left:5.1f} R:{packet.rpm_right:5.1f} | "
                      f"Acc:({packet.accel_x:5.2f}, {packet.accel_y:5.2f}, {packet.accel_z:5.2f}) | "
                      f"Mag:({packet.mag_x:6.3f}, {packet.mag_y:6.3f}, {packet.mag_z:6.3f})G")
            else:
                print("⏳ Waiting for data...")

            # Sleep 0.5s so the terminal doesn't scroll too fast
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
        stats = reader.get_stats()
        print(f"Final Stats: {stats['packets_received']} packets received, {stats['errors']} errors")
    finally:
        reader.close()
