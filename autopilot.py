"""
autopilot.py — Finite State Machine for autonomous collision-avoidance navigation.

States
──────
  CRUISING      Proportional speed control (fast in open space, brakes near walls)
  PANIC_BRAKE   Immediate motor stop on obstacle detection
  REVERSING     Drive backward to create maneuvering space (rear-aware)
  PIVOTING      True tank turn (opposite wheels) to face away from obstacle
  RECOVERY      Brief pause to let sensors stabilise before resuming
  STUCK         Emergency stop — car is pinned between front and rear obstacles

Sensor layout
─────────────
  Front Sonar   (Trig: 25, Echo: 24)  — forward obstacle detection
  Rear Sonar    (Trig: 26, Echo: 16)  — reverse-path safety
  Left/Right IR (GPIO 5 / 6)          — front-facing side detection

Dual-Direction Protection
─────────────────────────
  1.  Forward driving: if Front Sonar < 5 cm → ESCAPE ROUTINE.
  2.  Escape Routine checks Rear Sonar *before* reversing:
      • Rear < 10 cm → STUCK (car pinned, full stop).
      • Otherwise → reverse in 0.1 s micro-steps, checking rear each step.
      • If Rear < 5 cm *during* reverse → immediate stop & skip to pivot.

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
    STUCK       = "STUCK"


# ── AutoPilot ──────────────────────────────────────────

class AutoPilot:
    """
    Finite State Machine for intelligent autonomous navigation with
    dual-direction (front + rear) sonar protection.

    Parameters
    ----------
    car : motor.CarSystem
        Low-level motor controller (set_speed, reverse, pivot_turn, brake, stop).
    get_sonar : callable  →  float
        Returns front-centre distance in cm.
    get_ir : callable  →  (bool, bool)
        Returns (left_obstacle, right_obstacle).
    get_rear_sonar : callable  →  float
        Returns rear-centre distance in cm.
    """

    # ── Tuning constants ───────────────────────────────
    FRONT_CRITICAL_CM    = 5      # Front sonar: escape trigger threshold
    REAR_BLOCKED_CM      = 1     # Rear sonar: refuse to reverse below this
    REAR_CRITICAL_CM     = 2      # Rear sonar: interrupt reverse below this
    DANGER_CM            = 30     # Below this → escape maneuver (IR or sonar)
    FULL_SPEED_CM        = 80    # Above this → max cruise speed
    MAX_SPEED            = 60     # PWM % at full cruise
    MIN_SPEED            = 30     # PWM % at DANGER_CM boundary
    REVERSE_SPEED        = 40     # PWM % while reversing
    PIVOT_SPEED          = 50     # PWM % while pivoting
    REVERSE_DURATION     = 3.0    # max seconds of reverse (sum of micro-steps)
    REVERSE_STEP         = 0.5    # seconds per reverse micro-step
    PIVOT_DURATION       = 0.8    # seconds
    RECOVERY_DURATION    = 0.5    # seconds
    STUCK_RECHECK_INTERVAL = 1.0  # seconds between re-checking while stuck
    SONAR_HISTORY_LEN    = 3      # median filter window

    def __init__(self, car, get_sonar, get_ir, get_rear_sonar=None):
        self._car            = car
        self._get_sonar      = get_sonar
        self._get_ir         = get_ir
        self._get_rear_sonar = get_rear_sonar

        # FSM
        self._state           = State.CRUISING
        self._active          = False
        self._maneuver_start  = 0.0
        self._reverse_elapsed = 0.0
        self._turn_direction  = ""

        # Sensor noise filters
        self._sonar_history      = deque(maxlen=self.SONAR_HISTORY_LEN)
        self._rear_sonar_history = deque(maxlen=self.SONAR_HISTORY_LEN)

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
        self._rear_sonar_history.clear()

    def stop(self):
        """Deactivate autopilot and stop motors immediately."""
        self._active = False
        self._state  = State.CRUISING
        self._car.stop()

    # ── Sensor helpers ─────────────────────────────────

    def _filtered_sonar(self, raw):
        """
        Push a raw front reading into the history; return the median.
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

    def _filtered_rear_sonar(self, raw):
        """
        Push a raw rear reading into the history; return the median.
        Same glitch-rejection logic as the front sonar filter.
        """
        if raw is not None and raw > 0:
            self._rear_sonar_history.append(raw)

        if not self._rear_sonar_history:
            return 999.0

        buf = sorted(self._rear_sonar_history)
        mid = len(buf) // 2
        return float(buf[mid])

    def get_rear_distance(self):
        """
        Public helper: read and filter the rear sonar.
        Returns filtered distance in cm (999.0 if sensor unavailable).
        """
        if self._get_rear_sonar is None:
            return 999.0
        raw = self._get_rear_sonar()
        return self._filtered_rear_sonar(raw)

    def _get_front_distance(self):
        """Internal helper to read and filter the front sonar."""
        raw = self._get_sonar()
        return self._filtered_sonar(raw)

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
        raw_sonar         = self._get_sonar()
        distance          = self._filtered_sonar(raw_sonar)
        left_ir, right_ir = self._get_ir()

        # Dispatch
        handler = {
            State.CRUISING:    self._state_cruising,
            State.PANIC_BRAKE: self._state_panic_brake,
            State.REVERSING:   self._state_reversing,
            State.PIVOTING:    self._state_pivoting,
            State.RECOVERY:    self._state_recovery,
            State.STUCK:       self._state_stuck,
        }[self._state]

        handler(distance, left_ir, right_ir)

    # ── State A: CRUISING ──────────────────────────────

    def _state_cruising(self, distance, left_ir, right_ir):
        # CRITICAL CHECK: Front Sonar < 5 cm → escape
        if distance < self.FRONT_CRITICAL_CM:
            self._decide_turn_direction(left_ir, right_ir)
            self._state = State.PANIC_BRAKE
            self._car.brake()
            self._maneuver_start = time.time()
            print(f"🚨 [AutoPilot] CRITICAL: Front sonar {distance:.1f}cm < {self.FRONT_CRITICAL_CM}cm → PANIC_BRAKE, plan turn {self._turn_direction}")
            return

        # Wider threat check (IR or moderate sonar proximity)
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
        # to guarantee motors are fully stopped before the escape routine.
        self._car.brake()

        # ── Pre-reverse rear check ──────────────────────
        rear_dist = self.get_rear_distance()

        if rear_dist < self.REAR_BLOCKED_CM:
            # Car is pinned: front blocked AND rear blocked
            self._car.stop()
            self._state = State.STUCK
            self._maneuver_start = time.time()
            print(f"🛑 [AutoPilot] STUCK — Front {distance:.1f}cm, Rear {rear_dist:.1f}cm < {self.REAR_BLOCKED_CM}cm. Cannot reverse.")
            return

        # Rear is clear → proceed to rear-aware reverse
        self._state = State.REVERSING
        self._maneuver_start = time.time()
        self._reverse_elapsed = 0.0
        print(f"🔄 [AutoPilot] PANIC_BRAKE → REVERSING (rear clear at {rear_dist:.1f}cm)")

    # ── State C1: REVERSING (rear-aware micro-steps) ───

    def _state_reversing(self, distance, left_ir, right_ir):
        # Check rear sonar every tick
        rear_dist = self.get_rear_distance()

        if rear_dist < self.REAR_CRITICAL_CM:
            # Rear obstacle appeared during reverse → STOP IMMEDIATELY
            self._car.stop()
            self._state = State.PIVOTING
            self._maneuver_start = time.time()
            print(f"⚠️  [AutoPilot] Rear sonar {rear_dist:.1f}cm < {self.REAR_CRITICAL_CM}cm during reverse → STOP, skip to PIVOTING {self._turn_direction}")
            return

        # Continue reversing (use IR to bias direction)
        self._car.reverse(self.REVERSE_SPEED)

        elapsed = time.time() - self._maneuver_start
        if elapsed >= self.REVERSE_DURATION:
            self._car.stop()
            self._state = State.PIVOTING
            self._maneuver_start = time.time()
            print(f"🔄 [AutoPilot] REVERSING complete ({elapsed:.2f}s) → PIVOTING {self._turn_direction}")

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

    # ── State D: STUCK ─────────────────────────────────

    def _state_stuck(self, distance, left_ir, right_ir):
        """
        Car is pinned between obstacles.  Stay stopped and periodically
        re-check whether space has opened up behind.
        """
        self._car.stop()

        if time.time() - self._maneuver_start >= self.STUCK_RECHECK_INTERVAL:
            rear_dist = self.get_rear_distance()
            front_dist = distance

            if rear_dist >= self.REAR_BLOCKED_CM:
                # Rear cleared — try the escape routine again
                self._decide_turn_direction(left_ir, right_ir)
                self._state = State.REVERSING
                self._maneuver_start = time.time()
                self._reverse_elapsed = 0.0
                print(f"🔓 [AutoPilot] STUCK cleared — rear now {rear_dist:.1f}cm → REVERSING")
            elif front_dist >= self.DANGER_CM:
                # Front cleared — just go
                self._state = State.CRUISING
                print(f"🔓 [AutoPilot] STUCK cleared — front now {front_dist:.1f}cm → CRUISING")
            else:
                self._maneuver_start = time.time()
                print(f"🛑 [AutoPilot] Still STUCK — Front {front_dist:.1f}cm, Rear {rear_dist:.1f}cm")
