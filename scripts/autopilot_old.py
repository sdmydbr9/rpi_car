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
  UTURN         180° spin escape after repeated failed escape attempts

Sensor layout
─────────────
  Front Sonar   (Trig: 25, Echo: 24)  — forward obstacle detection
  Rear Sonar    (Trig: 26, Echo: 16)  — reverse-path safety

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

Smart Power Management (Anti-Stuck Logic)
─────────────────────────────────────────
  While cruising with throttle > 0, monitors sonar distance continuously.
  If distance hasn't changed by ±2 cm for 1.0 s, adds +5 % PWM boost per
  second (capped at 80 % PWM).  Resets to default speed once movement is
  detected (distance change > 5 cm).

Escalating Escape Maneuvers
───────────────────────────
  Tracks consecutive failed escape attempts (failed_attempts counter).
  Attempts 1-2: standard Reverse + Pivot.  Attempt 3+: executes a 180°
  U-Turn (high-speed spin for 0.8 s).  Counter resets when path clears
  (> 20 cm).
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
    UTURN       = "UTURN"


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
    get_rear_sonar : callable  →  float
        Returns rear-centre distance in cm.
    """

    # ── Tuning constants ───────────────────────────────
    FRONT_CRITICAL_CM    = 5      # Front sonar: escape trigger threshold
    REAR_BLOCKED_CM      = 3     # Rear sonar: refuse to reverse below this
    REAR_CRITICAL_CM     = 5      # Rear sonar: interrupt reverse below this
    DANGER_CM            = 40     # Below this → escape maneuver
    FULL_SPEED_CM        = 100    # Above this → max cruise speed
    MAX_SPEED            = 80     # PWM % at full cruise
    MIN_SPEED            = 30     # PWM % at DANGER_CM boundary
    REVERSE_SPEED        = 40     # PWM % while reversing
    PIVOT_SPEED          = 50    # PWM % while pivoting
    REVERSE_DURATION     = 1    # max seconds of reverse (sum of micro-steps)
    REVERSE_STEP         = 1.0    # seconds per reverse micro-step
    PIVOT_DURATION       = 1.0    # seconds
    RECOVERY_DURATION    = 1.0    # seconds
    STUCK_RECHECK_INTERVAL = 1.0  # seconds between re-checking while stuck
    SONAR_HISTORY_LEN    = 3      # median filter window

    # ── Smart Power Management constants ───────────
    STUCK_DISTANCE_THRESH = 2       # cm: distance change below this = "not moving"
    STUCK_TIME_THRESH     = 1.0     # seconds of no movement before boost
    STUCK_BOOST_STEP      = 5       # PWM % added per stuck interval
    STUCK_BOOST_MAX       = 80      # PWM % absolute cap for boost
    STUCK_MOVE_RESET      = 5       # cm: distance change above this = "moving again"

    # ── Escalating Escape constants ────────────────
    MAX_NORMAL_ESCAPES    = 2       # attempts before triggering U-turn
    UTURN_SPEED           = 75      # PWM % for U-turn spin  (was 70)
    UTURN_DURATION        = 1.2     # seconds (was 0.8)
    ESCAPE_CLEAR_CM       = 20      # cm: path considered clear → reset counter

    # ── Camera/Vision fusion constants ─────────────
    CAMERA_DANGER_CM      = 60      # camera-only early-warning threshold (cm)
    CAMERA_CRITICAL_CM    = 20      # camera-only hard-stop threshold (cm)
    CAMERA_TRUST_MIN_CM   = 10      # ignore camera distances below this (noise)

    # All tunable keys (used for serialisation)
    TUNING_KEYS = [
        'FRONT_CRITICAL_CM', 'REAR_BLOCKED_CM', 'REAR_CRITICAL_CM',
        'DANGER_CM', 'FULL_SPEED_CM', 'MAX_SPEED', 'MIN_SPEED',
        'REVERSE_SPEED', 'PIVOT_SPEED', 'REVERSE_DURATION', 'REVERSE_STEP',
        'PIVOT_DURATION', 'RECOVERY_DURATION', 'STUCK_RECHECK_INTERVAL',
        'SONAR_HISTORY_LEN', 'STUCK_DISTANCE_THRESH', 'STUCK_TIME_THRESH',
        'STUCK_BOOST_STEP', 'STUCK_BOOST_MAX', 'STUCK_MOVE_RESET',
        'MAX_NORMAL_ESCAPES', 'UTURN_SPEED', 'UTURN_DURATION', 'ESCAPE_CLEAR_CM',
        'CAMERA_DANGER_CM', 'CAMERA_CRITICAL_CM', 'CAMERA_TRUST_MIN_CM',
    ]

    def get_tuning(self):
        """Return current tuning constants as a dict."""
        return {k: getattr(self, k) for k in self.TUNING_KEYS}

    @classmethod
    def get_default_tuning(cls):
        """Return the original class-level default tuning constants."""
        # Read from the class itself (not an instance) to get compile-time defaults
        return {k: getattr(AutoPilot, k) for k in cls.TUNING_KEYS}

    def __init__(self, car, get_sonar, get_rear_sonar=None, get_camera_distance=None):
        self._car               = car
        self._get_sonar         = get_sonar
        self._get_rear_sonar    = get_rear_sonar
        self._get_camera_distance = get_camera_distance

        # FSM
        self._state           = State.CRUISING
        self._active          = False
        self._maneuver_start  = 0.0
        self._reverse_elapsed = 0.0
        self._turn_direction  = ""

        # Sensor noise filters
        self._sonar_history      = deque(maxlen=self.SONAR_HISTORY_LEN)
        self._rear_sonar_history = deque(maxlen=self.SONAR_HISTORY_LEN)

        # Smart Power Management (anti-stuck boost)
        self._stuck_boost        = 0       # accumulated PWM % on top of cruise speed
        self._last_cruise_dist   = None    # last distance reading while cruising
        self._stuck_timer        = 0.0     # time.time() when distance last changed

        # Escalating Escape Maneuvers
        self._failed_attempts    = 0       # consecutive blocked escape attempts

        # STUCK state active-escape tracking
        self._stuck_phase        = 0       # 0=pivot-L, 1=pivot-R, 2=uturn, 3=wait
        self._stuck_pivot_dir    = "left"  # alternates each U-turn cycle

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
        # Reset power management & escalation state
        self._stuck_boost      = 0
        self._last_cruise_dist = None
        self._stuck_timer      = 0.0
        self._failed_attempts  = 0
        self._stuck_phase      = 0

    def stop(self):
        """Deactivate autopilot and stop motors immediately."""
        self._active = False
        self._state  = State.CRUISING
        self._car.stop()
        self._stuck_boost     = 0
        self._failed_attempts = 0

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

    def _get_camera_distance_safe(self):
        """
        Read camera virtual distance; returns 999.0 if unavailable.
        Rejects values below CAMERA_TRUST_MIN_CM as noise.
        """
        if self._get_camera_distance is None:
            return 999.0
        try:
            d = self._get_camera_distance()
            if d is None or d < self.CAMERA_TRUST_MIN_CM:
                return 999.0
            return float(d)
        except Exception:
            return 999.0

    def _get_fused_front_distance(self):
        """
        Fused forward distance: minimum of sonar and camera.
        Conservative — reacts to whichever sensor sees an obstacle first.
        Camera provides earlier detection at medium range; sonar gives
        accurate close-range data.
        """
        sonar_dist = self._get_front_distance()
        camera_dist = self._get_camera_distance_safe()
        return min(sonar_dist, camera_dist)

    # ── Turn direction decision ────────────────────────

    def _decide_turn_direction(self):
        self._turn_direction = random.choice(["left", "right"])

    # ── Main update (call at ~20 Hz) ───────────────────

    def update(self):
        """
        Single tick of the FSM.  Should be called in a loop at ~20 Hz.
        Reads sensors, dispatches to the current state handler.
        Uses fused distance (sonar + camera) for forward-facing states
        (CRUISING, threat detection).  Reverse/stuck states use sonar only
        since the camera faces forward.
        """
        if not self._active:
            return

        # Read sensors — fused distance for forward states
        raw_sonar         = self._get_sonar()
        sonar_distance    = self._filtered_sonar(raw_sonar)
        camera_distance   = self._get_camera_distance_safe()
        # Use minimum of both sensors (conservative)
        distance          = min(sonar_distance, camera_distance)

        # Dispatch
        handler = {
            State.CRUISING:    self._state_cruising,
            State.PANIC_BRAKE: self._state_panic_brake,
            State.REVERSING:   self._state_reversing,
            State.PIVOTING:    self._state_pivoting,
            State.RECOVERY:    self._state_recovery,
            State.STUCK:       self._state_stuck,
            State.UTURN:       self._state_uturn,
        }[self._state]

        handler(distance)

    # ── State A: CRUISING ──────────────────────────────

    def _state_cruising(self, distance):
        # CRITICAL CHECK: Fused distance < 5 cm → escape
        if distance < self.FRONT_CRITICAL_CM:
            self._decide_turn_direction()
            self._state = State.PANIC_BRAKE
            self._car.brake()
            self._maneuver_start = time.time()
            print(f"🚨 [AutoPilot] CRITICAL: Fused distance {distance:.1f}cm < {self.FRONT_CRITICAL_CM}cm → PANIC_BRAKE, plan turn {self._turn_direction}")
            return

        # Camera-only critical check (object very close in camera)
        camera_dist = self._get_camera_distance_safe()
        if camera_dist < self.CAMERA_CRITICAL_CM:
            self._decide_turn_direction()
            self._state = State.PANIC_BRAKE
            self._car.brake()
            self._maneuver_start = time.time()
            print(f"📷 [AutoPilot] CAMERA CRITICAL: Object at {camera_dist:.1f}cm → PANIC_BRAKE, plan turn {self._turn_direction}")
            return

        # Wider threat check (moderate sonar/camera proximity)
        if distance < self.DANGER_CM:
            self._decide_turn_direction()
            self._state = State.PANIC_BRAKE
            self._car.brake()
            self._maneuver_start = time.time()
            if camera_dist < self.DANGER_CM:
                side = f"CAMERA {camera_dist:.0f}cm"
            else:
                side = f"SONAR {distance:.0f}cm"
            print(f"🚨 [AutoPilot] Threat detected ({side}) → PANIC_BRAKE, plan turn {self._turn_direction}")
            return

        # Proportional speed (based on fused distance)
        if distance >= self.FULL_SPEED_CM:
            base_speed = self.MAX_SPEED
        else:
            # Linear map: DANGER_CM → MIN_SPEED,  FULL_SPEED_CM → MAX_SPEED
            ratio = (distance - self.DANGER_CM) / (self.FULL_SPEED_CM - self.DANGER_CM)
            base_speed = self.MIN_SPEED + ratio * (self.MAX_SPEED - self.MIN_SPEED)
            base_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, base_speed))

        # ── Camera early-warning speed reduction ──────
        # If camera sees something sonar doesn't, proactively slow down
        if camera_dist < self.CAMERA_DANGER_CM and camera_dist < distance:
            cam_ratio = camera_dist / self.CAMERA_DANGER_CM  # 0.0 at 0cm, 1.0 at threshold
            cam_speed = self.MIN_SPEED + cam_ratio * (base_speed - self.MIN_SPEED)
            if cam_speed < base_speed:
                base_speed = cam_speed

        # ── Dynamic Power Boost (anti-stuck) ──────────
        now = time.time()

        if self._last_cruise_dist is None:
            # First tick — initialise tracking
            self._last_cruise_dist = distance
            self._stuck_timer = now

        delta = abs(distance - self._last_cruise_dist)

        if delta > self.STUCK_MOVE_RESET:
            # Car is clearly moving — reset boost
            if self._stuck_boost > 0:
                print(f"✅ [AutoPilot] Movement detected (Δ{delta:.1f}cm) → reset boost")
            self._stuck_boost = 0
            self._last_cruise_dist = distance
            self._stuck_timer = now
        elif delta > self.STUCK_DISTANCE_THRESH:
            # Some movement — update reference but keep current boost
            self._last_cruise_dist = distance
            self._stuck_timer = now
        else:
            # Distance unchanged — check if stuck long enough to boost
            if (now - self._stuck_timer) >= self.STUCK_TIME_THRESH:
                new_boost = self._stuck_boost + self.STUCK_BOOST_STEP
                cap = self.STUCK_BOOST_MAX - base_speed
                self._stuck_boost = max(0, min(new_boost, cap))
                self._stuck_timer = now  # restart interval
                self._last_cruise_dist = distance
                print(f"⚡ [AutoPilot] Stuck detected — boost now +{self._stuck_boost}% "
                      f"(total {base_speed + self._stuck_boost}%)")

        speed = min(base_speed + self._stuck_boost, self.STUCK_BOOST_MAX)

        self._car.set_speed(int(speed))
        self._car.set_steering(0)

    # ── State B: PANIC_BRAKE ───────────────────────────

    def _state_panic_brake(self, distance):
        # Brake is already applied on entry.  Hold for one cycle (~50 ms)
        # to guarantee motors are fully stopped before the escape routine.
        self._car.brake()

        # ── Pre-reverse rear check ──────────────────────
        rear_dist = self.get_rear_distance()

        # ── Escalation check ──────────────────────────
        self._failed_attempts += 1

        if self._failed_attempts > self.MAX_NORMAL_ESCAPES:
            # Too many failed escapes → perform U-turn
            self._state = State.UTURN
            self._maneuver_start = time.time()
            print(f"🔃 [AutoPilot] Escalation! {self._failed_attempts} attempts "
                  f"→ U-TURN ({self.UTURN_SPEED}% for {self.UTURN_DURATION}s)")
            return

        if rear_dist < self.REAR_BLOCKED_CM:
            # Can't reverse — but CAN pivot in place (zero-radius turn).
            # Skip straight to PIVOTING instead of sitting in STUCK.
            self._decide_turn_direction()
            self._state = State.PIVOTING
            self._maneuver_start = time.time()
            print(f"⚠️  [AutoPilot] Rear blocked ({rear_dist:.1f}cm < {self.REAR_BLOCKED_CM}cm) "
                  f"— skipping reverse, PIVOTING {self._turn_direction}")
            return

        # Rear is clear → proceed to rear-aware reverse
        self._state = State.REVERSING
        self._maneuver_start = time.time()
        self._reverse_elapsed = 0.0
        print(f"🔄 [AutoPilot] PANIC_BRAKE → REVERSING (rear clear at {rear_dist:.1f}cm, attempt {self._failed_attempts})")

    # ── State C1: REVERSING (rear-aware micro-steps) ───

    def _state_reversing(self, distance):
        # Check rear sonar every tick
        rear_dist = self.get_rear_distance()

        if rear_dist < self.REAR_CRITICAL_CM:
            # Rear obstacle appeared during reverse → STOP IMMEDIATELY
            self._car.stop()
            self._state = State.PIVOTING
            self._maneuver_start = time.time()
            return

        # Reverse with slight steering bias for better escape positioning.
        # Bias OPPOSITE to the planned pivot direction so the car swings
        # its nose away from the obstacle during the reverse.
        steer_bias = -25 if self._turn_direction == "left" else 25
        self._car.reverse_steer(self.REVERSE_SPEED, steer_bias)

        elapsed = time.time() - self._maneuver_start
        if elapsed >= self.REVERSE_DURATION:
            self._car.stop()
            self._state = State.PIVOTING
            self._maneuver_start = time.time()
            print(f"🔄 [AutoPilot] REVERSING complete ({elapsed:.2f}s) → PIVOTING {self._turn_direction}")

    # ── State C2: PIVOTING ─────────────────────────────

    def _state_pivoting(self, distance):
        self._car.pivot_turn(self._turn_direction, self.PIVOT_SPEED)

        if time.time() - self._maneuver_start >= self.PIVOT_DURATION:
            self._car.stop()
            self._state = State.RECOVERY
            self._maneuver_start = time.time()
            print("🔄 [AutoPilot] PIVOTING → RECOVERY")

    # ── State C3: RECOVERY ─────────────────────────────

    def _state_recovery(self, distance):
        # Car is stopped; sensors stabilise.
        if time.time() - self._maneuver_start >= self.RECOVERY_DURATION:
            # If path is clear, reset escalation counter
            if distance >= self.ESCAPE_CLEAR_CM:
                if self._failed_attempts > 0:
                    print(f"✅ [AutoPilot] Path clear ({distance:.1f}cm ≥ {self.ESCAPE_CLEAR_CM}cm) "
                          f"→ reset failed_attempts ({self._failed_attempts} → 0)")
                self._failed_attempts = 0
                self._state = State.CRUISING
                # Reset stuck-boost tracking for fresh cruise
                self._last_cruise_dist = None
                self._stuck_boost = 0
                print("✅ [AutoPilot] RECOVERY → CRUISING")
            else:
                # Path still blocked — re-enter escape sequence directly
                # instead of CRUISING (which would immediately re-trigger
                # PANIC_BRAKE and waste a full cycle).
                self._decide_turn_direction()
                self._state = State.PANIC_BRAKE
                self._car.brake()
                self._maneuver_start = time.time()
                print(f"⚠️  [AutoPilot] RECOVERY → path still blocked ({distance:.1f}cm) → PANIC_BRAKE")

    # ── State D: STUCK ─────────────────────────────────

    def _state_stuck(self, distance):
        """
        Car is pinned between obstacles.  Instead of passively waiting,
        actively cycle through escape maneuvers:
          Phase 0 — Pivot left   (1.5 s)
          Phase 1 — Pivot right  (1.5 s)
          Phase 2 — Aggressive U-turn spin
          Phase 3 — Brief pause, then restart the cycle
        After each phase, check whether front or rear has opened up.
        """
        now = time.time()
        rear_dist = self.get_rear_distance()
        front_dist = distance
        elapsed = now - self._maneuver_start

        # ── Quick-exit checks (run every tick) ─────────────────────
        if front_dist >= self.DANGER_CM:
            self._car.stop()
            self._failed_attempts = 0
            self._stuck_phase = 0
            self._state = State.CRUISING
            self._last_cruise_dist = None
            self._stuck_boost = 0
            print(f"🔓 [AutoPilot] STUCK cleared — front {front_dist:.1f}cm → CRUISING")
            return

        if rear_dist >= self.REAR_BLOCKED_CM + 3:
            self._car.stop()
            self._decide_turn_direction()
            self._state = State.REVERSING
            self._maneuver_start = now
            self._reverse_elapsed = 0.0
            self._stuck_phase = 0
            print(f"🔓 [AutoPilot] STUCK cleared — rear {rear_dist:.1f}cm → REVERSING")
            return

        # ── Phase 0: Pivot left ────────────────────────────────────
        if self._stuck_phase == 0:
            self._car.pivot_turn("left", self.PIVOT_SPEED)
            if elapsed >= 1.5:
                self._car.stop()
                self._stuck_phase = 1
                self._maneuver_start = now
                print("🔄 [AutoPilot] STUCK: pivot-left done → trying pivot-right")

        # ── Phase 1: Pivot right ───────────────────────────────────
        elif self._stuck_phase == 1:
            self._car.pivot_turn("right", self.PIVOT_SPEED)
            if elapsed >= 1.5:
                self._car.stop()
                self._stuck_phase = 2
                self._maneuver_start = now
                # Pick a U-turn direction (alternates each full cycle)
                self._stuck_pivot_dir = (
                    "right" if self._stuck_pivot_dir == "left" else "left"
                )
                print("🔄 [AutoPilot] STUCK: pivot-right done → trying U-turn")

        # ── Phase 2: Aggressive U-turn spin ────────────────────────
        elif self._stuck_phase == 2:
            self._car.pivot_turn(self._stuck_pivot_dir, self.UTURN_SPEED)
            if elapsed >= self.UTURN_DURATION:
                self._car.stop()
                self._stuck_phase = 3
                self._maneuver_start = now
                print(f"🔃 [AutoPilot] STUCK: U-turn ({self._stuck_pivot_dir}) done → brief pause")

        # ── Phase 3: Brief pause then restart cycle ────────────────
        elif self._stuck_phase == 3:
            self._car.stop()
            if elapsed >= 0.5:
                self._stuck_phase = 0
                self._maneuver_start = now
                print("🔄 [AutoPilot] STUCK: restarting escape cycle")

    # ── State E: U-TURN (escalated escape) ─────────────

    def _state_uturn(self, distance):
        """
        180° spin escape: triggered after MAX_NORMAL_ESCAPES consecutive
        failed reverse-and-pivot maneuvers.
        """
        self._car.pivot_turn(self._turn_direction, self.UTURN_SPEED)

        if time.time() - self._maneuver_start >= self.UTURN_DURATION:
            self._car.stop()
            self._state = State.RECOVERY
            self._maneuver_start = time.time()
            print(f"🔃 [AutoPilot] U-TURN complete → RECOVERY")
