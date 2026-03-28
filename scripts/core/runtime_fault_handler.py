"""
runtime_fault_handler.py — Real-time System Health Monitoring
═══════════════════════════════════════════════════════════════
Monitors runtime faults during autonomous operation and activates degraded modes.

Faults Tracked:
  • Stale Pico packets (UART communication timeout)
  • Camera frame freeze (vision system stalled)
  • Low battery voltage (power depletion warning)
  • High current draw (motor stall, overload, short)
  • Thermal throttling (CPU temperature critical)
  • Sensor disagreement (compass vs gyro, accel jolt, gyro spike)

When degraded mode is active, the vehicle limits speed, steering, and
disables autonomous features to ensure safe limp-home operation.
"""

import time
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List

from config import (
    BatteryMonitoring,
    CurrentMonitoring,
    ThermalMonitoring,
    SensorAgreement,
    LaserScanner,
    CameraMonitoring,
    PicoPacketFreshness,
    DegradedMode,
    SystemHealth,
)

log = logging.getLogger(__name__)


@dataclass
class FaultEvent:
    """Represents a detected fault."""
    name: str  # "stale_pico", "camera_freeze", etc.
    severity: str  # "warning" or "critical"
    timestamp: float = field(default_factory=time.time)
    details: str = ""
    value: Optional[float] = None  # For threshold breaches (actual value)
    threshold: Optional[float] = None  # For threshold breaches (limit)


class RuntimeFaultHandler:
    """
    Real-time fault detection and system health management.
    
    This handler is called from the main physics loop (~50 Hz) to check sensor
    health and packet freshness. It accumulates fault evidence and transitions
    the system into degraded mode when critical thresholds are reached.
    
    Usage:
        handler = RuntimeFaultHandler()
        
        # Call from physics loop
        health = handler.check_system_health(
            pico_reader=pico_reader_instance,  # Pico sensor bridge
            pkt=sensor_packet,  # Latest parsed sensor data
            camera_frame_count=latest_frame_number,  # Vision frame counter
            temp_c=cpu_temp_reading,  # CPU temp (or None to auto-read)
        )
        
        if health.degraded_mode_active:
            # Apply speed/steering limits, disable autopilot
            print(f"⚠️ DEGRADED MODE: {health.active_faults}")
    """

    def __init__(self):
        self.last_health = SystemHealth()
        
        # Fault tracking with hysteresis
        self._battery_low_triggered = False
        self._current_high_triggered = False
        self._thermal_throttle_triggered = False
        
        # Signal processing for disagreement detection
        self._compass_buffer = deque(maxlen=SensorAgreement.COMPASS_WINDOW_SIZE)
        self._gyro_buffer = deque(maxlen=SensorAgreement.COMPASS_WINDOW_SIZE)
        self._accel_buffer = deque(maxlen=2)  # Just last 2 samples for Δ
        self._gyro_rate_buffer = deque(maxlen=2)
        self._current_buffer = deque(maxlen=CurrentMonitoring.WINDOW_SIZE)
        
        # Camera frame tracking
        self._last_frame_count = -1
        self._stalled_frame_counter = 0
        self._last_camera_check = time.time()
        
        # Fault cooldowns (to avoid spamming logs for transient conditions)
        self._fault_cooldowns = {}
        self._COOLDOWN_S = 5.0


    def check_system_health(
        self,
        pico_reader=None,
        pkt=None,
        camera_frame_count: Optional[int] = None,
        temp_c: Optional[float] = None,
        compass_heading: Optional[float] = None,
        gyro_z: Optional[float] = None,
        accel_vec: Optional[tuple] = None,  # (x, y, z)
    ) -> SystemHealth:
        """
        Perform complete system health check.
        
        Parameters
        ----------
        pico_reader : PicoSensorReader
            Pico UART bridge instance (for packet freshness checks)
        pkt : SensorPacket or dict
            Latest parsed sensor packet from Pico
        camera_frame_count : int
            Current video frame counter (to detect freeze)
        temp_c : float
            CPU temperature in Celsius (auto-reads if None)
        compass_heading : float
            Compass heading in degrees (from pkt if None)
        gyro_z : float
            Gyro Z rate in deg/s (from pkt if None)
        accel_vec : tuple
            (accel_x, accel_y, accel_z) in g (from pkt if None)
        
        Returns
        -------
        SystemHealth
            Current system health state with active_faults list
        """
        
        health = SystemHealth()
        health.last_check_time = time.time()
        
        # ────────────────────────────────────────────────────────────────
        # 1. PICO PACKET FRESHNESS
        # ────────────────────────────────────────────────────────────────
        
        if pico_reader is not None:
            age_s = pico_reader.seconds_since_last_packet()
            
            if age_s > PicoPacketFreshness.CRITICAL_THRESHOLD_S:
                self._log_fault("stale_pico_critical", "critical",
                    f"Pico packet {age_s:.1f}s old (critical: >{PicoPacketFreshness.CRITICAL_THRESHOLD_S}s)")
                health.add_fault("PICO_PACKET_CRITICAL", 
                    f"{age_s:.1f}s stale")
                health.degraded_mode_active = True
                
            elif age_s > PicoPacketFreshness.STALE_THRESHOLD_S:
                self._log_fault("stale_pico_warning", "warning",
                    f"Pico packet {age_s:.1f}s old (warn: >{PicoPacketFreshness.STALE_THRESHOLD_S}s)")
                health.add_fault("PICO_PACKET_STALE",
                    f"{age_s:.1f}s old")
        
        # ────────────────────────────────────────────────────────────────
        # 2. CAMERA FRAME FREEZE
        # ────────────────────────────────────────────────────────────────
        
        if camera_frame_count is not None:
            now = time.time()
            
            # Check if frame counter has advanced
            if camera_frame_count == self._last_frame_count:
                self._stalled_frame_counter += 1
                
                if self._stalled_frame_counter > CameraMonitoring.STALLED_FRAME_COUNT:
                    self._log_fault("camera_freeze", "warning",
                        f"Camera stalled at frame {camera_frame_count}")
                    health.add_fault("CAMERA_FREEZE",
                        f"Frame {camera_frame_count}")
            else:
                self._stalled_frame_counter = 0
                self._last_frame_count = camera_frame_count
                self._last_camera_check = now
            
            # Check for absolute timeout (no frame received in a long time)
            if now - self._last_camera_check > CameraMonitoring.FRAME_TIMEOUT_S:
                self._log_fault("camera_timeout", "critical",
                    f"No camera frames for {now - self._last_camera_check:.1f}s")
                health.add_fault("CAMERA_TIMEOUT",
                    f">{CameraMonitoring.FRAME_TIMEOUT_S}s with no frames")
                health.degraded_mode_active = True
        
        # ────────────────────────────────────────────────────────────────
        # 3. BATTERY VOLTAGE
        # ────────────────────────────────────────────────────────────────
        
        if pkt is not None:
            battery_mv = getattr(pkt, 'adc_a0', None) or self._get_dict_value(pkt, 'battery_mv', 'adc_a0')
            
            if battery_mv is not None and battery_mv > -1:
                if battery_mv <= BatteryMonitoring.CRITICAL_LOW_MV:
                    self._log_fault("battery_critical", "critical",
                        f"Battery {battery_mv:.0f}mV (critical: <{BatteryMonitoring.CRITICAL_LOW_MV}mV)")
                    health.add_fault("BATTERY_CRITICAL",
                        f"{battery_mv:.0f}mV")
                    health.degraded_mode_active = True
                    self._battery_low_triggered = True
                
                elif battery_mv <= BatteryMonitoring.WARNING_LOW_MV and self._battery_low_triggered:
                    self._log_fault("battery_low", "warning",
                        f"Battery {battery_mv:.0f}mV (warning: <{BatteryMonitoring.WARNING_LOW_MV}mV)")
                    health.add_fault("BATTERY_LOW",
                        f"{battery_mv:.0f}mV")
                    health.degraded_mode_active = True
                
                elif battery_mv > BatteryMonitoring.WARNING_LOW_MV + BatteryMonitoring.HYSTERESIS_MV:
                    self._battery_low_triggered = False
        
        # ────────────────────────────────────────────────────────────────
        # 4. CURRENT DRAW (MOTOR OVERLOAD)
        # ────────────────────────────────────────────────────────────────
        
        if pkt is not None:
            current_mv = getattr(pkt, 'adc_a1', None) or self._get_dict_value(pkt, 'current_mv', 'adc_a1')
            
            if current_mv is not None and current_mv > -1:
                self._current_buffer.append(current_mv)
                
                if len(self._current_buffer) >= CurrentMonitoring.WINDOW_SIZE:
                    avg_current = sum(self._current_buffer) / len(self._current_buffer)
                    
                    if avg_current >= CurrentMonitoring.CRITICAL_HIGH_MA:
                        self._log_fault("current_critical", "critical",
                            f"Current {avg_current:.0f}mA (critical: >{CurrentMonitoring.CRITICAL_HIGH_MA}mA)")
                        health.add_fault("CURRENT_CRITICAL",
                            f"{avg_current:.0f}mA")
                        health.degraded_mode_active = True
                        self._current_high_triggered = True
                    
                    elif avg_current >= CurrentMonitoring.WARNING_HIGH_MA and self._current_high_triggered:
                        self._log_fault("current_high", "warning",
                            f"Current {avg_current:.0f}mA (warning: >{CurrentMonitoring.WARNING_HIGH_MA}mA)")
                        health.add_fault("CURRENT_HIGH",
                            f"{avg_current:.0f}mA")
                        health.degraded_mode_active = True
                    
                    elif avg_current < CurrentMonitoring.WARNING_HIGH_MA - CurrentMonitoring.HYSTERESIS_MA:
                        self._current_high_triggered = False
        
        # ────────────────────────────────────────────────────────────────
        # 5. THERMAL THROTTLING
        # ────────────────────────────────────────────────────────────────
        
        if temp_c is None:
            from config import read_thermal_temp_c
            temp_c = read_thermal_temp_c()
        
        if temp_c is not None:
            if temp_c >= ThermalMonitoring.CRITICAL_HIGH_C:
                self._log_fault("thermal_critical", "critical",
                    f"CPU temp {temp_c:.1f}°C (critical: >{ThermalMonitoring.CRITICAL_HIGH_C}°C)")
                health.add_fault("THERMAL_CRITICAL",
                    f"{temp_c:.1f}°C")
                health.degraded_mode_active = True
                self._thermal_throttle_triggered = True
            
            elif temp_c >= ThermalMonitoring.WARNING_THROTTLE_C and self._thermal_throttle_triggered:
                self._log_fault("thermal_throttle", "warning",
                    f"CPU temp {temp_c:.1f}°C (warning: >{ThermalMonitoring.WARNING_THROTTLE_C}°C)")
                health.add_fault("THERMAL_THROTTLE",
                    f"{temp_c:.1f}°C")
                health.degraded_mode_active = True
            
            elif temp_c < ThermalMonitoring.WARNING_THROTTLE_C - ThermalMonitoring.HYSTERESIS_C:
                self._thermal_throttle_triggered = False
        
        # ────────────────────────────────────────────────────────────────
        # 6. SENSOR DISAGREEMENT
        # ────────────────────────────────────────────────────────────────
        
        if compass_heading is not None:
            self._compass_buffer.append(compass_heading)
        
        if gyro_z is not None:
            self._gyro_buffer.append(gyro_z)
        
        if accel_vec is not None:
            self._accel_buffer.append(accel_vec)
            
            # Check for sudden acceleration changes
            if len(self._accel_buffer) >= 2:
                ax1, ay1, az1 = self._accel_buffer[-2]
                ax2, ay2, az2 = self._accel_buffer[-1]
                
                delta_ax = abs(ax2 - ax1)
                delta_ay = abs(ay2 - ay1)
                delta_az = abs(az2 - az1)
                max_delta = max(delta_ax, delta_ay, delta_az)
                
                if max_delta > SensorAgreement.ACCEL_MAX_JOLT_G:
                    self._log_fault("accel_jolt", "warning",
                        f"Accel jolt {max_delta:.2f}g (max: {SensorAgreement.ACCEL_MAX_JOLT_G}g)")
                    health.add_fault("ACCEL_ANOMALY",
                        f"Jolt {max_delta:.2f}g")
        
        if len(self._compass_buffer) >= SensorAgreement.COMPASS_WINDOW_SIZE and len(self._gyro_buffer) >= SensorAgreement.COMPASS_WINDOW_SIZE:
            avg_compass = sum(self._compass_buffer) / len(self._compass_buffer)
            avg_gyro = sum(self._gyro_buffer) / len(self._gyro_buffer)
            
            diff = abs(avg_compass - avg_gyro)
            if diff > 180:  # Wrap around
                diff = 360 - diff
            
            if diff > SensorAgreement.COMPASS_GYRO_MAX_DIFF_DEG:
                self._log_fault("sensor_disagreement", "warning",
                    f"Compass {avg_compass:.0f}° vs Gyro {avg_gyro:.0f}° (diff: {diff:.0f}°)")
                health.add_fault("SENSOR_DISAGREEMENT",
                    f"Compass/Gyro diff {diff:.0f}°")
        
        # ────────────────────────────────────────────────────────────────
        # 7. LASER MEASUREMENTS (sanity check)
        # ────────────────────────────────────────────────────────────────
        
        if pkt is not None:
            laser_mm = getattr(pkt, 'laser_mm', None) or self._get_dict_value(pkt, 'laser_mm', 'range_mm')
            
            if laser_mm is not None and laser_mm > 0:
                if laser_mm < LaserScanner.MINIMUM_RANGE_MM:
                    self._log_fault("laser_too_close", "warning",
                        f"Laser {laser_mm}mm (min: {LaserScanner.MINIMUM_RANGE_MM}mm)")
                    health.add_fault("LASER_SATURATED",
                        f"{laser_mm}mm")
                
                elif laser_mm > LaserScanner.SUSPICIOUS_RANGE_MM:
                    self._log_fault("laser_error", "warning",
                        f"Laser {laser_mm}mm (suspicious range)")
                    health.add_fault("LASER_ERROR",
                        f"{laser_mm}mm out-of-range")
        
        # Update last health state
        self.last_health = health
        
        return health


    def _get_dict_value(self, obj, *keys):
        """Try multiple key names for dict-like access."""
        for key in keys:
            if isinstance(obj, dict):
                if key in obj:
                    return obj[key]
            else:
                if hasattr(obj, key):
                    return getattr(obj, key)
        return None


    def _log_fault(self, fault_id: str, severity: str, message: str):
        """Log a fault with cooldown to avoid spam."""
        now = time.time()
        last_logged = self._fault_cooldowns.get(fault_id, 0)
        
        if now - last_logged > self._COOLDOWN_S:
            if severity == "critical":
                log.error(f"🔴 FAULT [{fault_id}] {message}")
            else:
                log.warning(f"🟡 FAULT [{fault_id}] {message}")
            self._fault_cooldowns[fault_id] = now


    @staticmethod
    def apply_degraded_mode_limits(motion_cmd: dict) -> dict:
        """
        Apply degraded mode speed/steering limits to a motion command.
        
        Parameters
        ----------
        motion_cmd : dict
            Keys: "throttle" (0-100), "steering_angle" (-35 to +35)
        
        Returns
        -------
        dict
            Limited command with same keys
        """
        
        cmd = motion_cmd.copy()
        
        # Limit speed
        throttle = cmd.get("throttle", 0) or 0
        throttle = max(0, min(DegradedMode.MAX_SPEED_PCT_DEGRADED, throttle))
        cmd["throttle"] = throttle
        
        # Limit steering angle
        steer = cmd.get("steering_angle", 0) or 0
        steer = max(-DegradedMode.MAX_STEER_ANGLE_DEGRADED, 
                   min(DegradedMode.MAX_STEER_ANGLE_DEGRADED, steer))
        cmd["steering_angle"] = steer
        
        return cmd


def get_fault_handler() -> RuntimeFaultHandler:
    """Singleton accessor for the fault handler."""
    global _fault_handler_instance
    if '_fault_handler_instance' not in globals():
        _fault_handler_instance = RuntimeFaultHandler()
    return _fault_handler_instance
