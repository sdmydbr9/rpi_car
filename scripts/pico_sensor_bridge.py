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
# MPU6050 (I2C IMU)
# =====================================================
MPU_ADDR = 0x68
mpu_ok = False

def mpu_write(reg, data):
    """Write to MPU6050 register with error handling."""
    try:
        i2c.writeto_mem(MPU_ADDR, reg, bytes([data]))
        return True
    except OSError as e:
        print(f"❌ MPU write failed: {e}")
        return False

def mpu_read(reg, length):
    """Read from MPU6050 register with error handling."""
    try:
        return i2c.readfrom_mem(MPU_ADDR, reg, length)
    except OSError as e:
        print(f"❌ MPU read failed: {e}")
        return None

# Wake up MPU6050 (clear sleep bit)
if mpu_write(0x6B, 0x00):
    time.sleep(0.1)
    mpu_ok = True
    print("✅ MPU6050 initialized (awakened)")
else:
    print("⚠️  MPU6050 failed to initialize")

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
# LM393 RPM ENCODER (INTERRUPT-DRIVEN)
# =====================================================
PULSES_PER_REV = 20
pulse_count = 0
last_rpm_time = time.ticks_ms()
rpm = 0.0

def rpm_callback(pin):
    """Interrupt handler for encoder pulses (falling edge)."""
    global pulse_count
    pulse_count += 1

lm393 = Pin(10, Pin.IN, Pin.PULL_UP)  # GPIO10 (pin 14)
lm393.irq(trigger=Pin.IRQ_FALLING, handler=rpm_callback)
print("✅ LM393 encoder initialized: GPIO10 (pin 14)")

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
        
        # --- MPU6050: Accel (3B-40) + Temp (41-42) + Gyro (43-48) ---
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
        
        temp_c = (temp / 340.0) + 36.53  # MPU6050 temperature formula
        
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
        
        # --- RPM Encoder (1-second rolling window) ---
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_rpm_time)
        
        if dt >= 1000:
            rpm = (pulse_count / PULSES_PER_REV) * 60.0
            pulse_count = 0
            last_rpm_time = now
        
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
            "rpm": round(rpm, 1),
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