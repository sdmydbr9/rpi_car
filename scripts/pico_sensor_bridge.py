# pico_sensor_bridge.py — Pico W Sensor Transmitter
# Ready for production: reads all sensors and streams JSON over UART @ 115200
# LED blinks on each data transmission

from machine import Pin, I2C, UART
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
# DIGITAL INPUTS (IR SENSORS)
# =====================================================
ir_left = Pin(8, Pin.IN)    # GPIO8 (pin 11)
ir_right = Pin(9, Pin.IN)   # GPIO9 (pin 12)
print("✅ IR sensors initialized: GPIO8 (left), GPIO9 (right)")

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
lm393_rr.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback_rr)
print("✅ LM393 encoder (rear right) initialized: GPIO10")

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
lm393_rl.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback_rl)
print("✅ LM393 encoder (rear left) initialized: GPIO15")

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
lm393_fr.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback_fr)
print("✅ LM393 encoder (front right) initialized: GPIO14")

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
        
        # --- IR Sensors (Active LOW: 0 = obstacle present) ---
        ir = {
            "left": ir_left.value() == 0,
            "right": ir_right.value() == 0
        }
        
        # --- RPM Encoders (100ms fast window + EMA smoothing) ---
        now = time.ticks_ms()
        
        dt_rr = time.ticks_diff(now, last_rpm_time_rr)
        if dt_rr >= RPM_WINDOW_MS:
            rpm_rr = (pulse_count_rr / PULSES_PER_REV) * (60000.0 / dt_rr)
            rpm_ema_rr = EMA_ALPHA * rpm_rr + (1.0 - EMA_ALPHA) * rpm_ema_rr
            pulse_count_rr = 0
            last_rpm_time_rr = now
        
        dt_rl = time.ticks_diff(now, last_rpm_time_rl)
        if dt_rl >= RPM_WINDOW_MS:
            rpm_rl = (pulse_count_rl / PULSES_PER_REV) * (60000.0 / dt_rl)
            rpm_ema_rl = EMA_ALPHA * rpm_rl + (1.0 - EMA_ALPHA) * rpm_ema_rl
            pulse_count_rl = 0
            last_rpm_time_rl = now
        
        dt_fr = time.ticks_diff(now, last_rpm_time_fr)
        if dt_fr >= RPM_WINDOW_MS:
            rpm_fr = (pulse_count_fr / PULSES_PER_REV) * (60000.0 / dt_fr)
            rpm_ema_fr = EMA_ALPHA * rpm_fr + (1.0 - EMA_ALPHA) * rpm_ema_fr
            pulse_count_fr = 0
            last_rpm_time_fr = now
        
        # --- Build JSON packet ---
        packet = {
            "ts": ts,
            "frame": frame_count,
            "accel": accel,
            "gyro": gyro,
            "temp_c": round(temp_c, 1),
            "laser_mm": laser_mm,
            "adc": ads_channels,  # New: 4-channel ADC readings
            "ir": ir,
            "rpm": {
                "rear_right": round(rpm_ema_rr, 1),
                "rear_left": round(rpm_ema_rl, 1),
                "front_right": round(rpm_ema_fr, 1)
            },
            "errors": error_count
        }
        
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
