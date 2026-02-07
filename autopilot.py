"""
autopilot.py — Finite State Machine for autonomous collision-avoidance navigation.

States
──────
  CRUISING      Proportional speed control (fast in open space, brakes near walls)
  PANIC_BRAKE   Immediate motor stop on obstacle detection
  REVERSING     Drive backward to create maneuvering space
  PIVOTING      True tank turn (opposite wheels) to face away from obstacle
  RECOVERY      Brief pause to let sensors stabilise before resuming

Sensor noise
────────────
  Sonar glitch filter: maintains a rolling window of recent readings.
  Single 0 cm / negative spikes are discarded; the median of the window
  is used as the "true" distance, eliminating single-reading glitches.
"""

import random
import time
from collections import deque
from enum import Enum


# ── FSM States ─────────────────────────────────────────

class State(Enum):
    CRUISING    = "CRUISING"
    PANIC_BRAKE = "PANIC_BRAKE"
    REVERSING   = "REVERSING"
    PIVOTING    = "PIVOTING"
    RECOVERY    = "RECOVERY"


# ── AutoPilot ──────────────────────────────────────────

class AutoPilot:
    """
    Finite State Machine for intelligent autonomous navigation.

    Parameters
    ----------
    car : motor.CarSystem
        Low-level motor controller (set_speed, reverse, pivot_turn, brake, stop).
    get_sonar : callable  →  float
        Returns front-centre distance in cm.
    get_ir : callable  →  (bool, bool)
        Returns (left_obstacle, right_obstacle).
    """

    # ── Tuning constants ───────────────────────────────
    DANGER_CM        = 30     # Below this → escape maneuver
    FULL_SPEED_CM    = 100    # Above this → max cruise speed
    MAX_SPEED        = 90     # PWM % at full cruise
    MIN_SPEED        = 40     # PWM % at DANGER_CM boundary
    REVERSE_SPEED    = 50     # PWM % while reversing
    PIVOT_SPEED      = 50     # PWM % while pivoting
    REVERSE_DURATION = 0.8    # seconds
    PIVOT_DURATION   = 0.4    # seconds
    RECOVERY_DURATION = 0.3   # seconds
    SONAR_HISTORY_LEN = 3     # median filter window

    def __init__(self, car, get_sonar, get_ir):
        self._car       = car
        self._get_sonar = get_sonar
        self._get_ir    = get_ir

        # FSM
        self._state           = State.CRUISING
        self._active          = False
        self._maneuver_start  = 0.0
        self._turn_direction  = ""

        # Sensor noise filter
        self._sonar_history = deque(maxlen=self.SONAR_HISTORY_LEN)

    # ── Properties ─────────────────────────────────────

    @property
    def state(self):
        return self._state

    @property
    def is_active(self):
        return self._active

    @property
    def turn_direction(self):
        return self._turn_direction

    # ── Start / Stop ───────────────────────────────────

    def start(self):
        """Activate autopilot; begins in CRUISING."""
        self._active = True
        self._state  = State.CRUISING
        self._sonar_history.clear()

    def stop(self):
        """Deactivate autopilot and stop motors immediately."""
        self._active = False
        self._state  = State.CRUISING
        self._car.stop()

    # ── Sensor noise filter ────────────────────────────

    def _filtered_sonar(self, raw):
        """
        Push a raw reading into the history; return the median.
        Rejects invalid values (≤ 0) — they are not added to the window.
        If the window is empty, returns a safe fallback (999 cm).
        """
        if raw is not None and raw > 0:
            self._sonar_history.append(raw)

        if not self._sonar_history:
            return 999.0  # safe fallback: assume wide open

        buf = sorted(self._sonar_history)
        mid = len(buf) // 2
        return float(buf[mid])

    # ── Turn direction decision ────────────────────────

    def _decide_turn_direction(self, left_ir, right_ir):
        if left_ir and not right_ir:
            self._turn_direction = "right"
        elif right_ir and not left_ir:
            self._turn_direction = "left"
        else:
            # Both blocked or only sonar (center hit) → random
            self._turn_direction = random.choice(["left", "right"])

    # ── Main update (call at ~20 Hz) ───────────────────

    def update(self):
        """
        Single tick of the FSM.  Should be called in a loop at ~20 Hz.
        Reads sensors, dispatches to the current state handler.
        """
        if not self._active:
            return

        # Read sensors
        raw_sonar      = self._get_sonar()
        distance       = self._filtered_sonar(raw_sonar)
        left_ir, right_ir = self._get_ir()

        # Dispatch
        handler = {
            State.CRUISING:    self._state_cruising,
            State.PANIC_BRAKE: self._state_panic_brake,
            State.REVERSING:   self._state_reversing,
            State.PIVOTING:    self._state_pivoting,
            State.RECOVERY:    self._state_recovery,
        }[self._state]

        handler(distance, left_ir, right_ir)

    # ── State A: CRUISING ──────────────────────────────

    def _state_cruising(self, distance, left_ir, right_ir):
        # Threat check → escape
        if distance < self.DANGER_CM or left_ir or right_ir:
            self._decide_turn_direction(left_ir, right_ir)
            self._state = State.PANIC_BRAKE
            self._car.brake()
            self._maneuver_start = time.time()
            side = "LEFT_IR" if left_ir else ("RIGHT_IR" if right_ir else f"SONAR {distance:.0f}cm")
            print(f"🚨 [AutoPilot] Threat detected ({side}) → PANIC_BRAKE, plan turn {self._turn_direction}")
            return

        # Proportional speed
        if distance >= self.FULL_SPEED_CM:
            speed = self.MAX_SPEED
        else:
            # Linear map: DANGER_CM → MIN_SPEED,  FULL_SPEED_CM → MAX_SPEED
            ratio = (distance - self.DANGER_CM) / (self.FULL_SPEED_CM - self.DANGER_CM)
            speed = self.MIN_SPEED + ratio * (self.MAX_SPEED - self.MIN_SPEED)
            speed = max(self.MIN_SPEED, min(self.MAX_SPEED, speed))

        self._car.set_speed(int(speed))
        self._car.set_steering(0)

    # ── State B: PANIC_BRAKE ───────────────────────────

    def _state_panic_brake(self, distance, left_ir, right_ir):
        # Brake is already applied on entry.  Hold for one cycle (~50 ms)
        # to guarantee motors are fully stopped before reversing.
        self._car.brake()
        # Transition immediately on next tick
        self._state = State.REVERSING
        self._maneuver_start = time.time()
        print("🔄 [AutoPilot] PANIC_BRAKE → REVERSING")

    # ── State C1: REVERSING ────────────────────────────

    def _state_reversing(self, distance, left_ir, right_ir):
        self._car.reverse(self.REVERSE_SPEED)

        if time.time() - self._maneuver_start >= self.REVERSE_DURATION:
            self._car.stop()
            self._state = State.PIVOTING
            self._maneuver_start = time.time()
            print(f"🔄 [AutoPilot] REVERSING → PIVOTING {self._turn_direction}")

    # ── State C2: PIVOTING ─────────────────────────────

    def _state_pivoting(self, distance, left_ir, right_ir):
        self._car.pivot_turn(self._turn_direction, self.PIVOT_SPEED)

        if time.time() - self._maneuver_start >= self.PIVOT_DURATION:
            self._car.stop()
            self._state = State.RECOVERY
            self._maneuver_start = time.time()
            print("🔄 [AutoPilot] PIVOTING → RECOVERY")

    # ── State C3: RECOVERY ─────────────────────────────

    def _state_recovery(self, distance, left_ir, right_ir):
        # Car is stopped; sensors stabilise.
        if time.time() - self._maneuver_start >= self.RECOVERY_DURATION:
            self._state = State.CRUISING
            print("✅ [AutoPilot] RECOVERY → CRUISING")
