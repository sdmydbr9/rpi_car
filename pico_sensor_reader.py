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
    
    # MPU6050 (accel in g, gyro in deg/s)
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    temp_c: float
    
    # VL53L0X laser (mm)
    laser_mm: int
    
    # ADS1115 ADC (mV)
    adc_a0: float  # Channel A0 (e.g., battery voltage)
    adc_a1: float  # Channel A1 (e.g., current sense)
    adc_a2: float  # Channel A2 (spare)
    adc_a3: float  # Channel A3 (spare)
    
    # IR sensors (bool)
    ir_left: bool
    ir_right: bool
    
    # LM393 encoder (RPM)
    rpm: float
    
    # Error counter
    errors: int


class PicoSensorReader:
    """
    Reads JSON sensor packets from Pico over UART.
    Provides thread-safe access to latest sensor data.
    """
    
    def __init__(self, port='/dev/ttyS0', baudrate=115200, buffer_size=10):
        """
        Initialize Pico sensor reader.
        
        Parameters
        ----------
        port : str
            Serial port (default /dev/ttyS0 for Pi 4B mini-UART)
        baudrate : int
            Baud rate (default 115200)
        buffer_size : int
            Size of rolling packet buffer
        """
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._last_packet = None
        self._packet_buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._running = False
        self._error_count = 0
        self._packet_count = 0
        self._read_thread = None
        
        # Try to connect
        self._connect()
    
    def _connect(self):
        """Attempt to establish serial connection."""
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✅ Pico sensor reader connected: {self.port} @ {self.baudrate} baud")
            self._running = True
            
            # Start background read thread
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
                        packet = SensorPacket(
                            timestamp_ms=raw.get('ts', 0),
                            frame=raw.get('frame', 0),
                            accel_x=raw['accel']['x'],
                            accel_y=raw['accel']['y'],
                            accel_z=raw['accel']['z'],
                            gyro_x=raw['gyro']['x'],
                            gyro_y=raw['gyro']['y'],
                            gyro_z=raw['gyro']['z'],
                            temp_c=raw.get('temp_c', 0.0),
                            laser_mm=raw.get('laser_mm', -1),
                            adc_a0=raw.get('adc', {}).get('A0', -1),
                            adc_a1=raw.get('adc', {}).get('A1', -1),
                            adc_a2=raw.get('adc', {}).get('A2', -1),
                            adc_a3=raw.get('adc', {}).get('A3', -1),
                            ir_left=raw['ir']['left'],
                            ir_right=raw['ir']['right'],
                            rpm=raw.get('rpm', 0.0),
                            errors=raw.get('errors', 0)
                        )
                        
                        with self._lock:
                            self._last_packet = packet
                            self._packet_buffer.append(packet)
                            self._packet_count += 1
                    
                    except (json.JSONDecodeError, KeyError, TypeError) as e:
                        self._error_count += 1
                        print(f"⚠️  Parse error: {e} | Data: {line[:60]}")
                        
            except Exception as e:
                self._error_count += 1
                print(f"⚠️  Read error: {e}")
                time.sleep(0.01)
    
    def get_latest(self):
        """
        Get the most recent sensor packet.
        
        Returns
        -------
        SensorPacket or None
            Latest packet, or None if not yet received
        """
        with self._lock:
            return self._last_packet
    
    def get_buffer(self):
        """
        Get last N packets (for averaging/filtering).
        
        Returns
        -------
        list[SensorPacket]
            Rolling buffer of packets (most recent last)
        """
        with self._lock:
            return list(self._packet_buffer)
    
    def is_connected(self):
        """Check if reader is actively receiving data."""
        with self._lock:
            return self._running and self._last_packet is not None
    
    def get_stats(self):
        """Get reader statistics."""
        with self._lock:
            return {
                "packets_received": self._packet_count,
                "errors": self._error_count,
                "connected": self._running
            }
    
    def close(self):
        """Shutdown reader and close serial port."""
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self._serial:
            self._serial.close()
        print("🛑 Pico sensor reader closed")


# ============================================================
# CONVENIENCE FUNCTIONS (for integration with existing code)
# ============================================================

_global_reader = None

def init_pico_reader(port='/dev/ttyS0'):
    """
    Initialize global Pico sensor reader.
    Call once at startup.
    """
    global _global_reader
    _global_reader = PicoSensorReader(port)
    return _global_reader

def get_sensor_packet():
    """Get latest sensor packet from Pico."""
    if _global_reader:
        return _global_reader.get_latest()
    return None

def get_gyro_z():
    """Get gyro Z rate (deg/s) - for heading integration."""
    packet = get_sensor_packet()
    return packet.gyro_z if packet else 0.0

def get_accel_xyz():
    """Get accelerometer data (g)."""
    packet = get_sensor_packet()
    if packet:
        return packet.accel_x, packet.accel_y, packet.accel_z
    return 0.0, 0.0, 0.0

def get_laser_distance_mm():
    """Get laser distance (mm), or -1 if invalid."""
    packet = get_sensor_packet()
    return packet.laser_mm if packet else -1

def get_laser_distance_cm():
    """Get laser distance (cm), or -1 if invalid."""
    mm = get_laser_distance_mm()
    return round(mm / 10.0, 1) if mm > 0 else -1

def get_ir_sensors():
    """Get IR obstacle flags (left_detected, right_detected)."""
    packet = get_sensor_packet()
    if packet:
        return packet.ir_left, packet.ir_right
    return False, False

def get_rpm():
    """Get wheel RPM."""
    packet = get_sensor_packet()
    return packet.rpm if packet else 0.0


def get_adc_voltage_mv(channel='A0'):
    """
    Get ADC channel voltage in millivolts.
    
    Parameters
    ----------
    channel : str
        'A0', 'A1', 'A2', or 'A3'
    
    Returns
    -------
    float
        Voltage in mV, or -1 if invalid
    """
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
    """
    Get battery voltage (assumes voltage divider on A0).
    
    Parameters
    ----------
    divider_ratio : float
        Divider ratio (default 5.0 for standard DC 0-25V sensor module)
        Calculated from onboard SMD resistors: (30k + 7.5k) / 7.5k = 5.0
    
    Returns
    -------
    float
        Battery voltage in V, or -1 if unavailable
    """
    adc_mv = get_adc_voltage_mv('A0')
    if adc_mv < 0:
        return -1
    return (adc_mv / 1000.0) * divider_ratio


def get_current_sense(shunt_ohms=0.1):
    """
    Get current from shunt resistor on A1.
    
    Parameters
    ----------
    shunt_ohms : float
        Shunt resistance (default 0.1Ω for typical 100mΩ resistor)
        Current (A) = ADC_voltage (V) / R_shunt (Ω)
    
    Returns
    -------
    float
        Current in Amps, or -1 if unavailable
    """
    adc_mv = get_adc_voltage_mv('A1')
    if adc_mv < 0:
        return -1
    adc_v = adc_mv / 1000.0
    return adc_v / shunt_ohms


if __name__ == "__main__":
    # Test standalone
    print("\n" + "="*60)
    print("  PICO SENSOR READER TEST")
    print("="*60 + "\n")
    
    reader = PicoSensorReader()
    
    print("Waiting for packets...\n")
    time.sleep(2)
    
    packet = reader.get_latest()
    if packet:
        print(f"✅ Latest packet received:")
        print(f"   Frame: {packet.frame}")
        print(f"   Accel: ({packet.accel_x:.2f}, {packet.accel_y:.2f}, {packet.accel_z:.2f}) g")
        print(f"   Gyro:  ({packet.gyro_x:.1f}, {packet.gyro_y:.1f}, {packet.gyro_z:.1f}) °/s")
        print(f"   Laser: {packet.laser_mm} mm")
        # Updated divider ratio to 5 for the 0-25V sensor
        print(f"   ADC A0 (battery): {packet.adc_a0:.1f} mV → {packet.adc_a0 / 1000 * 5:.1f} V (if divider=5)")
        print(f"   ADC A1 (current): {packet.adc_a1:.1f} mV → {packet.adc_a1 / 1000 / 0.1:.2f} A (if shunt=0.1Ω)")
        print(f"   ADC A2: {packet.adc_a2:.1f} mV")
        print(f"   ADC A3: {packet.adc_a3:.1f} mV")
        print(f"   IR:    L={packet.ir_left}, R={packet.ir_right}")
        print(f"   RPM:   {packet.rpm:.1f}")
    else:
        print("❌ No packets received yet")
    
    stats = reader.get_stats()
    print(f"\nStats: {stats['packets_received']} packets, {stats['errors']} errors")
    
    reader.close()