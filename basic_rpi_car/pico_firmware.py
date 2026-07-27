"""Pico firmware for the standalone ``basic_rpi_car`` controller.

Copy this file to the Pico as ``main.py``. It produces no unsolicited UART
traffic. The only replies are PONG and BAT:<millivolts> after explicit queries.
"""

from machine import I2C, Pin, PWM, UART
import time


UART_BAUD = 115200
MAX_PWM_DUTY = 95
WATCHDOG_TIMEOUT_MS = 300
MOTOR_TICK_MS = 10
MAX_PWM_DELTA_PER_TICK = 3
DIRECTION_CHANGE_DWELL_MS = 150

STEER_LEFT_PW = 940
STEER_CENTER_PW = 1440
STEER_RIGHT_PW = 2150

ADS1115_ADDRESS = 0x48


uart = UART(0, baudrate=UART_BAUD, tx=Pin(0), rx=Pin(1), txbuf=256)

steer_servo = PWM(Pin(15))
steer_servo.freq(50)

motor_left_pwm = PWM(Pin(10))
motor_left_pwm.freq(1000)
motor_left_in1 = Pin(17, Pin.OUT)
motor_left_in2 = Pin(12, Pin.OUT)

motor_right_pwm = PWM(Pin(16))
motor_right_pwm.freq(1000)
motor_right_in1 = Pin(13, Pin.OUT)
motor_right_in2 = Pin(14, Pin.OUT)

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
ads_available = ADS1115_ADDRESS in i2c.scan()


_steer_left_pw = STEER_LEFT_PW
_steer_center_pw = STEER_CENTER_PW
_steer_right_pw = STEER_RIGHT_PW

_target_left = 0
_target_right = 0
_applied_left = 0.0
_applied_right = 0.0
_forward = True
_last_direction = None
_last_drive_command_ms = time.ticks_ms()

_direction_dwell = False
_direction_dwell_started_ms = 0

_brake_active = False
_brake_started_ms = 0
_brake_start_left = 0.0
_brake_start_right = 0.0

_estop_latched = False
_uart_buffer = ""
_last_motor_tick_ms = time.ticks_ms()


def clamp(value, minimum, maximum):
    return minimum if value < minimum else maximum if value > maximum else value


def set_steering_pulse(pulse_us):
    pulse = clamp(int(pulse_us), _steer_left_pw, _steer_right_pw)
    steer_servo.duty_ns(pulse * 1000)


def set_steering_calibration(left_pw, center_pw, right_pw):
    global _steer_left_pw, _steer_center_pw, _steer_right_pw
    left_pw = int(left_pw)
    center_pw = int(center_pw)
    right_pw = int(right_pw)
    if not 500 <= left_pw < center_pw < right_pw <= 2500:
        return False
    _steer_left_pw = left_pw
    _steer_center_pw = center_pw
    _steer_right_pw = right_pw
    set_steering_pulse(center_pw)
    return True


def set_motor_direction(forward):
    if forward:
        motor_left_in1.value(0)
        motor_left_in2.value(1)
        motor_right_in1.value(0)
        motor_right_in2.value(1)
    else:
        motor_left_in1.value(1)
        motor_left_in2.value(0)
        motor_right_in1.value(1)
        motor_right_in2.value(0)


def set_left_pwm(duty):
    motor_left_pwm.duty_u16(
        int(clamp(duty, 0, MAX_PWM_DUTY) / 100 * 65535)
    )


def set_right_pwm(duty):
    motor_right_pwm.duty_u16(
        int(clamp(duty, 0, MAX_PWM_DUTY) / 100 * 65535)
    )


def coast_stop():
    global _target_left, _target_right, _applied_left, _applied_right
    global _direction_dwell, _brake_active
    _target_left = 0
    _target_right = 0
    _applied_left = 0.0
    _applied_right = 0.0
    set_left_pwm(0)
    set_right_pwm(0)
    motor_left_in1.value(0)
    motor_left_in2.value(0)
    motor_right_in1.value(0)
    motor_right_in2.value(0)
    _direction_dwell = False
    _brake_active = False


def start_brake(restart=False):
    global _brake_active, _brake_started_ms
    global _brake_start_left, _brake_start_right
    global _target_left, _target_right
    if _brake_active and not restart:
        return
    _target_left = 0
    _target_right = 0
    _brake_start_left = _applied_left
    _brake_start_right = _applied_right
    _brake_started_ms = time.ticks_ms()
    _brake_active = True


def ramp_value(applied, target):
    difference = target - applied
    if abs(difference) > MAX_PWM_DELTA_PER_TICK:
        difference = (
            MAX_PWM_DELTA_PER_TICK
            if difference > 0
            else -MAX_PWM_DELTA_PER_TICK
        )
    return clamp(applied + difference, 0, MAX_PWM_DUTY)


def motor_tick(now_ms):
    global _applied_left, _applied_right, _last_direction
    global _direction_dwell, _direction_dwell_started_ms, _brake_active

    if _brake_active:
        elapsed = time.ticks_diff(now_ms, _brake_started_ms)
        motor_left_in1.value(1)
        motor_left_in2.value(1)
        motor_right_in1.value(1)
        motor_right_in2.value(1)
        if elapsed < 100:
            progress = elapsed / 100.0
            set_left_pwm(
                _brake_start_left
                + (MAX_PWM_DUTY - _brake_start_left) * progress
            )
            set_right_pwm(
                _brake_start_right
                + (MAX_PWM_DUTY - _brake_start_right) * progress
            )
        else:
            set_left_pwm(MAX_PWM_DUTY)
            set_right_pwm(MAX_PWM_DUTY)
            if _estop_latched and elapsed >= 150:
                coast_stop()
        return

    if _direction_dwell:
        if (
            time.ticks_diff(now_ms, _direction_dwell_started_ms)
            < DIRECTION_CHANGE_DWELL_MS
        ):
            return
        _direction_dwell = False
        _last_direction = _forward

    if _last_direction is not None and _last_direction != _forward:
        set_left_pwm(0)
        set_right_pwm(0)
        _applied_left = 0.0
        _applied_right = 0.0
        _direction_dwell = True
        _direction_dwell_started_ms = now_ms
        return

    set_motor_direction(_forward)
    _last_direction = _forward
    _applied_left = ramp_value(_applied_left, _target_left)
    _applied_right = ramp_value(_applied_right, _target_right)
    set_left_pwm(_applied_left)
    set_right_pwm(_applied_right)


def read_battery_adc_mv():
    if not ads_available:
        return None
    try:
        # A0 single-ended, +/-4.096 V PGA, one-shot conversion, 128 SPS.
        i2c.writeto_mem(ADS1115_ADDRESS, 0x01, bytes([0xC3, 0x83]))
        time.sleep_ms(10)
        raw = i2c.readfrom_mem(ADS1115_ADDRESS, 0x00, 2)
        value = (raw[0] << 8) | raw[1]
        if value & 0x8000:
            value -= 65536
        return round(value * 0.125, 1)
    except Exception:
        return None


def apply_drive(direction, pwm, steering_pulse):
    global _target_left, _target_right, _forward, _last_drive_command_ms
    if _estop_latched:
        return
    direction = direction.upper()
    pwm = clamp(int(pwm), 0, MAX_PWM_DUTY)
    if direction not in ("F", "N", "R"):
        return
    if _brake_active:
        coast_stop()
    if direction == "N":
        pwm = 0
    elif direction == "F":
        _forward = True
    else:
        _forward = False
    _target_left = pwm
    _target_right = pwm
    set_steering_pulse(steering_pulse)
    _last_drive_command_ms = time.ticks_ms()


def handle_command(line):
    global _estop_latched, _last_drive_command_ms
    if line.startswith("D:"):
        try:
            direction, pwm, steering_pulse = line[2:].split(",", 2)
            apply_drive(direction, pwm, steering_pulse)
        except Exception:
            pass
    elif line == "B":
        if not _estop_latched:
            start_brake()
        _last_drive_command_ms = time.ticks_ms()
    elif line == "S":
        coast_stop()
        _last_drive_command_ms = time.ticks_ms()
    elif line == "E":
        if not _estop_latched:
            _estop_latched = True
            start_brake(restart=True)
    elif line == "ERST":
        if (
            _target_left == 0
            and _target_right == 0
            and _applied_left == 0
            and _applied_right == 0
            and not _brake_active
        ):
            coast_stop()
            _estop_latched = False
    elif line.startswith("SC:"):
        try:
            left_pw, center_pw, right_pw = line[3:].split(",", 2)
            set_steering_calibration(left_pw, center_pw, right_pw)
        except Exception:
            pass
    elif line == "PING":
        uart.write(b"PONG\n")
    elif line == "BAT?":
        if (
            _target_left
            or _target_right
            or _applied_left
            or _applied_right
            or _brake_active
            or _direction_dwell
        ):
            uart.write(b"BAT:ERR\n")
        else:
            battery_mv = read_battery_adc_mv()
            if battery_mv is None:
                uart.write(b"BAT:ERR\n")
            else:
                uart.write(
                    ("BAT:{:.1f}\n".format(battery_mv)).encode("ascii")
                )


def read_uart_commands():
    global _uart_buffer
    while uart.any():
        chunk = uart.read(1)
        if not chunk:
            return
        character = chunk.decode("ascii", "ignore")
        if character == "\n":
            command = _uart_buffer.strip()
            _uart_buffer = ""
            if command:
                handle_command(command)
        elif character != "\r":
            _uart_buffer += character
            if len(_uart_buffer) > 80:
                _uart_buffer = ""


coast_stop()
set_steering_pulse(_steer_center_pw)

while True:
    read_uart_commands()
    now = time.ticks_ms()

    if (
        (
            _target_left
            or _target_right
            or (_brake_active and not _estop_latched)
        )
        and time.ticks_diff(now, _last_drive_command_ms)
        > WATCHDOG_TIMEOUT_MS
    ):
        coast_stop()

    if time.ticks_diff(now, _last_motor_tick_ms) >= MOTOR_TICK_MS:
        _last_motor_tick_ms = now
        motor_tick(now)

    time.sleep_ms(1)
