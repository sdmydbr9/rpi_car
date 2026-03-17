# pico_sensor_bridge.py — Pico W Sensor Transmitter
# Ready for production: reads all sensors and streams JSON over UART @ 115200
# LED blinks on each data transmission

from machine import Pin, I2C, UART, PWM
import machine
import time
import struct
import json

print("\n" + "="*60)
print("  PICO W SENSOR BRIDGE — Starting...")
print("="*60 + "\n")

# =====================================================
# LED SETUP (Pico W built-in LED on GPIO25)
# =====================================================
led = Pin("LED", Pin.OUT)  # Pico W has built-in LED
led.off()

def blink_led(count=1, delay_ms=50):
    """Blink LED to signal data transmission."""
    for _ in range(count):
        led.on()
        time.sleep_ms(delay_ms)
        led.off()
        time.sleep_ms(delay_ms // 2)

# =====================================================
# UART SETUP (to Raspberry Pi)
# =====================================================
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
print("✅ UART0 initialized: TX=GPIO0, RX=GPIO1 @ 115200 baud")

# =====================================================
# PAN-TILT SERVO SETUP (PWM on GP2 + GP3)
# =====================================================
pan_servo = PWM(Pin(2))
tilt_servo = PWM(Pin(3))
pan_servo.freq(50)
tilt_servo.freq(50)

PAN_CENTER  = 90
TILT_CENTER = 90
PAN_MIN  = 60;  PAN_MAX  = 120
TILT_MIN = 60;  TILT_MAX = 120
SERVO_MIN_DUTY = 2500
SERVO_MAX_DUTY = 7500

_pan_angle  = PAN_CENTER
_tilt_angle = TILT_CENTER

def _servo_clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def _set_servo(pwm, angle):
    angle = _servo_clamp(angle, 0, 180)
    duty = int(SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * (angle / 180.0))
    pwm.duty_u16(duty)

def _apply_pan_tilt():
    _set_servo(pan_servo, _pan_angle)
    _set_servo(tilt_servo, _tilt_angle)

_apply_pan_tilt()
print(f"✅ Pan-tilt servos initialized: GP2 (pan), GP3 (tilt) — centered at {PAN_CENTER}°")

_uart_buf = ""

def _check_uart_commands():
    """Non-blocking: read incoming bytes and parse PT commands.
    Protocol:  PT:<pan>,<tilt>\n   (angles as integers)
    Also:      PC\n                (center both servos)
    """
    global _uart_buf, _pan_angle, _tilt_angle
    while uart.any():
        ch = uart.read(1)
        if ch is None:
            break
        ch = ch.decode('ascii', 'ignore')
        if ch == '\n':
            line = _uart_buf.strip()
            _uart_buf = ""
            if line.startswith("PT:"):
                try:
                    parts = line[3:].split(",")
                    p = int(parts[0])
                    t = int(parts[1])
                    _pan_angle  = _servo_clamp(p, PAN_MIN, PAN_MAX)
                    _tilt_angle = _servo_clamp(t, TILT_MIN, TILT_MAX)
                    _apply_pan_tilt()
                except Exception:
                    pass
            elif line == "PC":
                _pan_angle  = PAN_CENTER
                _tilt_angle = TILT_CENTER
                _apply_pan_tilt()
        else:
            _uart_buf += ch
            if len(_uart_buf) > 64:
                _uart_buf = ""

# =====================================================
# I2C SETUP (MPU6050 + VL53L0X + ADS1115)
# =====================================================
i2c = I2C(
    0,
    scl=Pin(5),       # GPIO5 (pin 7)
    sda=Pin(4),       # GPIO4 (pin 6)
    freq=100000       # 100 kHz (lowered from 400k for noise immunity)
)

time.sleep(1)
devices = i2c.scan()
device_addrs = [hex(d) for d in devices]
print(f"✅ I2C devices found: {device_addrs}")
print(f"   Expected: 0x68 (MPU6050), 0x29 (VL53L0X), 0x48 or 0x49 (ADS1115)\n")

# =====================================================
# MPU6500 (I2C IMU)
# =====================================================
MPU_ADDR = 0x68
mpu_ok = False

def mpu_write(reg, data):
    """Write to MPU6500 register with error handling."""
    try:
        i2c.writeto_mem(MPU_ADDR, reg, bytes([data]))
        return True
    except OSError as e:
        print(f"❌ MPU write failed: {e}")
        return False

def mpu_read(reg, length):
    """Read from MPU6500 register with error handling."""
    try:
        return i2c.readfrom_mem(MPU_ADDR, reg, length)
    except OSError as e:
        print(f"❌ MPU read failed: {e}")
        return None

# Wake up MPU6500 (clear sleep bit)
if mpu_write(0x6B, 0x00):
    time.sleep(0.1)
    mpu_ok = True
    print("✅ MPU6500 initialized (awakened)")
else:
    print("⚠️  MPU6500 failed to initialize")

# =====================================================
# VL53L0X (I2C laser)
# =====================================================
VL53_ADDR = 0x29
vl53_ok = False

def vl53_init():
    """Initialize VL53L0X."""
    try:
        i2c.writeto_mem(VL53_ADDR, 0x00, b'\x01')  # Soft reset
        time.sleep_ms(50)
        return True
    except OSError:
        return False

def vl53_single_measure():
    """Read single distance measurement from VL53L0X."""
    try:
        # Trigger ranging
        i2c.writeto_mem(VL53_ADDR, 0x00, b'\x01')
        time.sleep_ms(35)  # Wait for measurement (typical ~33ms)
        
        # Read result (registers 0x14-0x1F contain status and distance)
        data = i2c.readfrom_mem(VL53_ADDR, 0x14, 12)
        distance_mm = (data[10] << 8) | data[11]
        
        # Filter invalid readings
        if 30 <= distance_mm <= 2000:
            return distance_mm
        else:
            return -1
    except OSError:
        return -1

vl53_ok = vl53_init()
if vl53_ok:
    print("✅ VL53L0X initialized")
else:
    print("⚠️  VL53L0X not responding (continuing without laser)")

# =====================================================
# QMC5883L MAGNETOMETER (I2C 0x0D)
# OSR=512, RNG=8G (3000 LSB/Gauss), ODR=200Hz, Continuous
# =====================================================
QMC_ADDR = 0x0D
qmc_ok = False

def qmc_init():
    """Initialize QMC5883L in continuous measurement mode."""
    try:
        # Soft reset
        i2c.writeto_mem(QMC_ADDR, 0x0B, b'\x01')
        time.sleep_ms(10)
        # Control register 1: OSR=512, RNG=8G, ODR=200Hz, Continuous mode
        i2c.writeto_mem(QMC_ADDR, 0x09, b'\x1D')
        time.sleep_ms(100)
        print(f"✅ QMC5883L magnetometer initialized (I2C 0x{QMC_ADDR:02x})")
        return True
    except OSError as e:
        print(f"⚠️  QMC5883L not found at 0x{QMC_ADDR:02x}: {e}")
        return False

def qmc_read():
    """
    Read magnetometer axes. Returns (x, y, z) in Gauss.
    Registers 0x00-0x05: X_L, X_H, Y_L, Y_H, Z_L, Z_H (little-endian).
    Scale: 3000 LSB/Gauss at RNG=8G.
    """
    try:
        data = i2c.readfrom_mem(QMC_ADDR, 0x00, 6)
        x, y, z = struct.unpack('<hhh', data)
        return round(x / 3000.0, 4), round(y / 3000.0, 4), round(z / 3000.0, 4)
    except OSError:
        return None, None, None

qmc_ok = qmc_init()

# =====================================================
# ADS1115 (I2C 4-channel 16-bit ADC)
# =====================================================
ADS_ADDR = 0x48  # Change this if your ADS1115 uses a different address
ads_ok = False

# ADS1115 Registers
ADS_REG_CONVERSION = 0x00
ADS_REG_CONFIG = 0x01

def ads_init():
    """Initialize ADS1115."""
    try:
        # Test read (should return default config or current value)
        i2c.readfrom_mem(ADS_ADDR, ADS_REG_CONFIG, 2)
        print(f"✅ ADS1115 initialized (I2C 0x{ADS_ADDR:02x})")
        return True
    except OSError as e:
        print(f"⚠️  ADS1115 not found at 0x{ADS_ADDR:02x}: {e}")
        return False

def ads_read_channel(channel):
    """
    Read single-shot from ADS1115 channel (0-3) in Single-Ended mode.
    Returns voltage in mV.
    """
    if channel < 0 or channel > 3:
        return -1
    
    try:
        # FIX: Add 4 to the channel to set "Single-Ended" mode (MUX = 100, 101, 110, 111)
        mux = (4 + channel) << 4  
        
        # FIX: Correctly map the high byte
        # 0x80 (Start) | mux (Channel) | 0x02 (PGA ±4.096V) | 0x01 (Single-shot Mode)
        config_high = 0x80 | mux | 0x02 | 0x01 
        config_low = 0x83  # DR=128 SPS, Disable Comparator
        
        # Write config
        i2c.writeto_mem(ADS_ADDR, ADS_REG_CONFIG, bytes([config_high, config_low]))
        
        # Wait for conversion (1/128 sps ≈ 8ms)
        time.sleep_ms(10)
        
        # Read conversion result
        data = i2c.readfrom_mem(ADS_ADDR, ADS_REG_CONVERSION, 2)
        raw = struct.unpack('>h', data)[0]  # Big-endian signed 16-bit
        
        # FIX: Use all 16 bits! LSB for ±4.096V is 0.125 mV per bit
        voltage_mv = raw * 0.125 
        
        # Clean up floating noise near 0V
        if voltage_mv < 0:
            voltage_mv = 0.0
            
        return round(voltage_mv, 1)
    
    except OSError:
        return -1

ads_ok = ads_init()

# =====================================================
# LM393 RPM ENCODERS (INTERRUPT-DRIVEN)
# Software debounce: 2ms lockout per channel (supports up to ~1500 RPM)
# EMA smoothing: 100ms fast window + exponential moving average
# =====================================================
PULSES_PER_REV = 20
DEBOUNCE_US = 2000       # 2ms lockout in microseconds
RPM_WINDOW_MS = 100      # 100ms calculation window (10 Hz)
EMA_ALPHA = 0.3          # EMA smoothing factor (0.0–1.0, higher = more responsive)

# Rear right encoder (original) — GPIO10
pulse_count_rr = 0
last_rpm_time_rr = time.ticks_ms()
rpm_rr = 0.0
rpm_ema_rr = 0.0
_last_irq_us_rr = 0

def rpm_callback_rr(pin):
    """Interrupt handler for rear right encoder (falling edge, 2ms debounce)."""
    global pulse_count_rr, _last_irq_us_rr
    now = time.ticks_us()
    if time.ticks_diff(now, _last_irq_us_rr) >= DEBOUNCE_US:
        pulse_count_rr += 1
        _last_irq_us_rr = now

lm393_rr = Pin(10, Pin.IN, Pin.PULL_UP)  # GPIO10 (pin 14)
lm393_rr.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback_rr, hard=True)
print("✅ LM393 encoder (rear right) initialized: GPIO10 [hard IRQ]")

# Rear left encoder — GPIO15
pulse_count_rl = 0
last_rpm_time_rl = time.ticks_ms()
rpm_rl = 0.0
rpm_ema_rl = 0.0
_last_irq_us_rl = 0

def rpm_callback_rl(pin):
    """Interrupt handler for rear left encoder (falling edge, 2ms debounce)."""
    global pulse_count_rl, _last_irq_us_rl
    now = time.ticks_us()
    if time.ticks_diff(now, _last_irq_us_rl) >= DEBOUNCE_US:
        pulse_count_rl += 1
        _last_irq_us_rl = now

lm393_rl = Pin(15, Pin.IN, Pin.PULL_UP)  # GPIO15
lm393_rl.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback_rl, hard=True)
print("✅ LM393 encoder (rear left) initialized: GPIO15 [hard IRQ]")

# Front right encoder — GPIO14
pulse_count_fr = 0
last_rpm_time_fr = time.ticks_ms()
rpm_fr = 0.0
rpm_ema_fr = 0.0
_last_irq_us_fr = 0

def rpm_callback_fr(pin):
    """Interrupt handler for front right encoder (falling edge, 2ms debounce)."""
    global pulse_count_fr, _last_irq_us_fr
    now = time.ticks_us()
    if time.ticks_diff(now, _last_irq_us_fr) >= DEBOUNCE_US:
        pulse_count_fr += 1
        _last_irq_us_fr = now

lm393_fr = Pin(14, Pin.IN, Pin.PULL_UP)  # GPIO14
lm393_fr.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback_fr, hard=True)
print("✅ LM393 encoder (front right) initialized: GPIO14 [hard IRQ]")

# =====================================================
# MAIN TRANSMISSION LOOP (50 Hz ≈ 20ms per frame)
# =====================================================
print("\n" + "="*60)
print("  🚀 TRANSMITTING SENSOR DATA...")
print("="*60 + "\n")

frame_count = 0
error_count = 0

while True:
    try:
        ts = time.ticks_ms()
        
        # --- MPU6500: Accel (3B-40) + Temp (41-42) + Gyro (43-48) ---
        if not mpu_ok:
            time.sleep_ms(20)
            continue
        
        raw = mpu_read(0x3B, 14)  # 14 bytes: Ax, Ay, Az, Temp, Gx, Gy, Gz
        if raw is None:
            error_count += 1
            time.sleep_ms(20)
            continue
        
        ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", raw)
        
        accel = {
            "x": round(ax / 16384.0, 3),
            "y": round(ay / 16384.0, 3),
            "z": round(az / 16384.0, 3)
        }
        
        gyro = {
            "x": round(gx / 131.0, 2),
            "y": round(gy / 131.0, 2),
            "z": round(gz / 131.0, 2)
        }
        
        temp_c = (temp / 333.87) + 21.0  # MPU6500 temperature formula
        
        # --- VL53L0X: Distance ---
        laser_mm = vl53_single_measure() if vl53_ok else -1
        
        # --- QMC5883L: Magnetometer (Gauss) ---
        if qmc_ok:
            mx, my, mz = qmc_read()
            mag_data = {"x": mx, "y": my, "z": mz} if mx is not None else {"x": 0.0, "y": 0.0, "z": 0.0}
        else:
            mag_data = {"x": 0.0, "y": 0.0, "z": 0.0}

        # --- ADS1115: 4-channel ADC (voltage in mV) ---
        ads_channels = {}
        if ads_ok:
            # Common uses:
            #   A0 = Battery voltage (divider: actual_V = adc_V * 5 for 0-25V sensor)
            #   A1 = Current sense (shunt: current_A = adc_mV / 100 for 100mΩ shunt)
            #   A2 = Spare analog input
            #   A3 = Spare analog input
            ads_channels["A0"] = ads_read_channel(0)  # mV
            ads_channels["A1"] = ads_read_channel(1)  # mV
            ads_channels["A2"] = ads_read_channel(2)  # mV
            ads_channels["A3"] = ads_read_channel(3)  # mV
        else:
            ads_channels = {"A0": -1, "A1": -1, "A2": -1, "A3": -1}
        
        # --- RPM Encoders (100ms fast window + EMA smoothing) ---
        # Atomic read-and-reset: disable IRQs to prevent losing pulses
        now = time.ticks_ms()
        
        dt_rr = time.ticks_diff(now, last_rpm_time_rr)
        if dt_rr >= RPM_WINDOW_MS:
            irq_state = machine.disable_irq()
            _pc_rr = pulse_count_rr
            pulse_count_rr = 0
            machine.enable_irq(irq_state)
            rpm_rr = (_pc_rr / PULSES_PER_REV) * (60000.0 / dt_rr)
            rpm_ema_rr = EMA_ALPHA * rpm_rr + (1.0 - EMA_ALPHA) * rpm_ema_rr
            last_rpm_time_rr = now
        
        dt_rl = time.ticks_diff(now, last_rpm_time_rl)
        if dt_rl >= RPM_WINDOW_MS:
            irq_state = machine.disable_irq()
            _pc_rl = pulse_count_rl
            pulse_count_rl = 0
            machine.enable_irq(irq_state)
            rpm_rl = (_pc_rl / PULSES_PER_REV) * (60000.0 / dt_rl)
            rpm_ema_rl = EMA_ALPHA * rpm_rl + (1.0 - EMA_ALPHA) * rpm_ema_rl
            last_rpm_time_rl = now
        
        dt_fr = time.ticks_diff(now, last_rpm_time_fr)
        if dt_fr >= RPM_WINDOW_MS:
            irq_state = machine.disable_irq()
            _pc_fr = pulse_count_fr
            pulse_count_fr = 0
            machine.enable_irq(irq_state)
            rpm_fr = (_pc_fr / PULSES_PER_REV) * (60000.0 / dt_fr)
            rpm_ema_fr = EMA_ALPHA * rpm_fr + (1.0 - EMA_ALPHA) * rpm_ema_fr
            last_rpm_time_fr = now
        
        # --- Build JSON packet ---
        packet = {
            "ts": ts,
            "frame": frame_count,
            "accel": accel,
            "gyro": gyro,
            "temp_c": round(temp_c, 1),
            "mag": mag_data,
            "laser_mm": laser_mm,
            "adc": ads_channels,  # New: 4-channel ADC readings
            "rpm": {
                "rear_right": round(rpm_ema_rr, 1),
                "rear_left": round(rpm_ema_rl, 1),
                "front_right": round(rpm_ema_fr, 1)
            },
            "errors": error_count
        }
        
        # --- Check for incoming pan-tilt commands ---
        _check_uart_commands()
        
        # --- Transmit over UART ---
        json_str = json.dumps(packet) + "\n"
        uart.write(json_str)
        
        # --- Blink LED to signal transmission ---
        blink_led(count=1, delay_ms=30)
        
        frame_count += 1
        
        # Sleep remainder of 20ms frame (50 Hz output)
        time.sleep_ms(20)
        
    except Exception as e:
        # Send error packet
        error_packet = {
            "error": str(e),
            "frame": frame_count,
            "ts": time.ticks_ms()
        }
        try:
            uart.write(json.dumps(error_packet) + "\n")
        except:
            pass
        
        error_count += 1
        print(f"❌ Exception: {e}")
        
        # LED double-blink for error
        blink_led(count=2, delay_ms=100)
        
        time.sleep_ms(100)
