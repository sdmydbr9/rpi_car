"""
config.py — Centralized Configuration for Pi Car
─────────────────────────────────────────────────
Single source of truth for hardware limits, sensor thresholds, and fault detection parameters.
Consolidates all scattered configuration that was previously hardcoded throughout the codebase.
"""

import os
from dataclasses import dataclass
from typing import Optional


# ══════════════════════════════════════════════════════════════════════════════
# HARDWARE & COMMUNICATION
# ══════════════════════════════════════════════════════════════════════════════

class PicoBridge:
    """Pico UART bridge configuration."""
    PORT = "/dev/ttyS0"
    BAUDRATE = 115200
    TIMEOUT = 0.1
    TX_BUFFER_SIZE = 512
    MAX_PACKET_SIZE = 220  # JSON bytes to avoid buffer truncation
    FRAME_RATE_HZ = 50  # Pico sends at 50 Hz (20ms frames)


class Encoders:
    """Wheel encoder configuration."""
    PPR = 330 * 4  # Pico counts 4× per hall cycle (both edges)
    WHEEL_DIAMETER_M = 0.075  # 75mm wheels
    WHEEL_CIRCUMFERENCE_M = 0.2356  # π × diameter


class Steering:
    """Ackermann steering servo PWM limits (from physical testing)."""
    CENTER_PW = 1440  # µs — straight ahead
    LEFT_PW = 940     # µs — max left lock
    RIGHT_PW = 2150   # µs — max right lock
    LOCK_ANGLE_DEG = 35  # Physical steering lock angle


class PanTilt:
    """Pan/tilt gimbal servo limits."""
    PAN_CENTER = 90
    TILT_CENTER = 90
    PAN_MIN = 60
    PAN_MAX = 120
    TILT_MIN = 60
    TILT_MAX = 120


# ══════════════════════════════════════════════════════════════════════════════
# SENSOR THRESHOLDS & LIMITS
# ══════════════════════════════════════════════════════════════════════════════

class BatteryMonitoring:
    """Battery voltage thresholds."""
    ADC_A0_CHANNEL = 0  # ADS1115 channel A0 reads battery voltage
    
    # Voltage levels (mV)
    NOMINAL_MV = 7400  # 2S LiPo nominal (2 × 3.7V)
    CRITICAL_LOW_MV = 6400  # ~3.2V per cell (stop immediately)
    WARNING_LOW_MV = 6800  # ~3.4V per cell (degrade mode)
    VOLTAGE_DIVIDER_RATIO = 1.0  # ADC reads direct battery voltage
    
    # Hysteresis to prevent oscillation
    HYSTERESIS_MV = 200


class CurrentMonitoring:
    """Current sense monitoring."""
    ADC_A1_CHANNEL = 1  # ADS1115 channel A1 reads current sense
    
    # Current thresholds (mA equivalent)
    NOMINAL_MA = 1000  # Typical cruise current
    WARNING_HIGH_MA = 3000  # Excessive current (motor stall, high load)
    CRITICAL_HIGH_MA = 4000  # Motor protection threshold
    
    # Moving average window
    WINDOW_SIZE = 10  # samples for smoothing
    
    # Hysteresis
    HYSTERESIS_MA = 300


class ThermalMonitoring:
    """CPU temperature and thermal throttling."""
    THERMAL_ZONE_PATH = "/sys/class/thermal/thermal_zone0/temp"
    CPU_FREQCAP_PATH = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"
    
    # Temperature thresholds (°C)
    NOMINAL_MAX_C = 65  # Safe operating limit
    WARNING_THROTTLE_C = 75  # Thermal throttling likely
    CRITICAL_HIGH_C = 85  # Aggressive throttling / shutdown risk
    
    # Hysteresis
    HYSTERESIS_C = 5


class SensorAgreement:
    """Sensor disagreement detection."""
    
    # Compass vs Gyro heading disagreement
    COMPASS_GYRO_MAX_DIFF_DEG = 45  # Alert if they differ by >45°
    COMPASS_WINDOW_SIZE = 10  # samples for comparison
    
    # Accelerometer consistency (rapid changes indicate sensor malfunction)
    ACCEL_MAX_JOLT_G = 2.0  # Alert if Δ accel > 2g in one frame
    
    # Gyroscope consistency
    GYRO_MAX_CHANGE_DPS = 500  # Alert if Δ gyro > 500°/s in one frame


class LaserScanner:
    """Laser (VL53L0X) thresholds."""
    OUT_OF_RANGE_MM = 2000  # VL53L0X saturation
    MINIMUM_RANGE_MM = 50  # Too close
    SUSPICIOUS_RANGE_MM = 10000  # Clearly erroneous
    MAX_AGE_S = 1.0  # Stale measurement


class CameraMonitoring:
    """Camera freeze detection."""
    FRAME_TIMEOUT_S = 2.0  # Alert if no frame for 2 seconds
    STALLED_FRAME_COUNT = 5  # Same frame number for 5+ checks


# ══════════════════════════════════════════════════════════════════════════════
# PICO PACKET FRESHNESS
# ══════════════════════════════════════════════════════════════════════════════

class PicoPacketFreshness:
    """Stale Pico packet detection."""
    STALE_THRESHOLD_S = 1.0  # Packet older than 1 second is stale
    CRITICAL_THRESHOLD_S = 3.0  # Packet older than 3 seconds is critical


# ══════════════════════════════════════════════════════════════════════════════
# DEGRADED MODE BEHAVIOR
# ══════════════════════════════════════════════════════════════════════════════

class DegradedMode:
    """Fallback behavior when faults are detected."""
    
    # Speed reduction in degraded mode
    MAX_SPEED_PCT_DEGRADED = 30  # Limit to 30% duty cycle
    MAX_ACCEL_DEGRADED = 10  # Smoother acceleration ramp (duty % per 20ms frame)
    
    # Steering reduction in degraded mode
    MAX_STEER_ANGLE_DEGRADED = 15  # Limit to ±15° steering angle
    
    # Disable autonomous features
    DISABLE_AUTOPILOT = True
    DISABLE_HUNTER_MODE = True
    DISABLE_CAMERA_TRACKING = True
    
    # Sensor fallbacks
    USE_GYRO_WITHOUT_COMPASS = True  # Use gyro alone if compass fails
    USE_LASER_WITHOUT_CAMERA = True  # Use laser alone if camera fails
    USE_OPEN_LOOP_WITHOUT_ENCODERS = True  # Fallback to PWM if encoder stalls


# ══════════════════════════════════════════════════════════════════════════════
# AUTONOMOUS DRIVING
# ══════════════════════════════════════════════════════════════════════════════

class Autopilot:
    """Autonomous driving parameters."""
    OBSTACLE_SWERVE_ANGLE = 85  # Degrees to swerve when obstacle detected
    OBSTACLE_STOP_DISTANCE_CM = 30  # cm minimum before stopping


class FollowTarget:
    """Visual target following parameters."""
    MIN_CONFIDENCE = 0.5  # Minimum detection confidence (0.0-1.0)
    FRAME_TIMEOUT_S = 0.5  # Lose target if not seen for 0.5s


# ══════════════════════════════════════════════════════════════════════════════
# WHEEL SYNCHRONIZATION (2WD Rear)
# ══════════════════════════════════════════════════════════════════════════════

class WheelSync:
    """Rear wheel speed synchronization (PID control)."""
    TARGET_RPM_MAX = 200  # Max target RPM for rear wheels
    SYNC_ERROR_THRESHOLD_RPM = 10  # Alert if L/R differ by >10 RPM
    SYNC_WINDOW_SIZE = 5  # samples for moving average


# ══════════════════════════════════════════════════════════════════════════════
# LOGGING & TELEMETRY
# ══════════════════════════════════════════════════════════════════════════════

class Logging:
    """Logging configuration."""
    LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "rover_logs")
    LOG_LEVEL = "INFO"  # DEBUG, INFO, WARNING, ERROR


class Telemetry:
    """Telemetry postman configuration."""
    SUBMIT_INTERVAL_S = 2.0  # Submit stats every 2 seconds
    INCLUDE_POWER_FIELDS = True
    INCLUDE_POSITION_FIELDS = True
    INCLUDE_SENSOR_FIELDS = True


# ══════════════════════════════════════════════════════════════════════════════
# RUNTIME FAULT OPTIONS
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SystemHealth:
    """Current system health snapshot."""
    healthy: bool = True
    active_faults: list = None  # List of fault strings
    degraded_mode_active: bool = False
    last_check_time: float = 0.0
    
    def __post_init__(self):
        if self.active_faults is None:
            self.active_faults = []
    
    def add_fault(self, fault_name: str, details: str = ""):
        """Record a detected fault."""
        fault_str = f"{fault_name}: {details}" if details else fault_name
        if fault_str not in self.active_faults:
            self.active_faults.append(fault_str)
        self.healthy = False
    
    def clear_fault(self, fault_name: str):
        """Clear a specific fault."""
        self.active_faults = [f for f in self.active_faults if not f.startswith(fault_name)]
        if not self.active_faults:
            self.healthy = True


# ══════════════════════════════════════════════════════════════════════════════
# UTILITY FUNCTIONS
# ══════════════════════════════════════════════════════════════════════════════

def get_log_dir() -> str:
    """Get the logging directory, creating it if needed."""
    os.makedirs(Logging.LOG_DIR, exist_ok=True)
    return Logging.LOG_DIR


def read_thermal_temp_c() -> Optional[float]:
    """Read current CPU temperature from thermal zone."""
    try:
        with open(ThermalMonitoring.THERMAL_ZONE_PATH) as f:
            millicelsius = int(f.read().strip())
            return millicelsius / 1000.0
    except Exception:
        return None


def read_cpu_freq_mhz() -> Optional[float]:
    """Read current CPU frequency cap."""
    try:
        with open(ThermalMonitoring.CPU_FREQCAP_PATH) as f:
            hz = int(f.read().strip())
            return hz / 1e6
    except Exception:
        return None
