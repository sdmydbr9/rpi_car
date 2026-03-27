"""
wheel_sync.py — Closed-Loop Wheel Speed Synchronization Controller

Transparently intercepts motor PWM commands and applies per-wheel PID
correction using live RPM feedback from rear encoders (via Pico bridge).

Hardware context (Hiwonder Ackermann Metal Chassis):
  - 2 encoder-geared rear-drive motors (RL, RR) — matched gear ratio
  - 1 LD-1501MG Ackermann front steering servo (non-driven front axle)
  - 2 rear encoders: Rear-Left (RL), Rear-Right (RR)

Algorithm: PID + feedforward with back-calculation anti-windup,
           asymmetric slew limiting, and EMA-filtered RPM readings.
           Ported from motor_test.py RPM-LOCK mode (proven on hardware).

Integration: Called from CarSystem._set_raw_motors() — the single
             chokepoint for ALL motor output (manual, autopilot, hunter).
"""

import time
import threading


# ──────────────────────────────────────────────────────────────
# Tuning Constants
# ──────────────────────────────────────────────────────────────

# PID gains (shared across all wheels — integral absorbs per-wheel differences)
SYNC_KP = 0.012       # Proportional (gentle — feedforward does heavy lifting)
SYNC_KI = 0.004       # Integral (trims steady-state error)
SYNC_KD = 0.012       # Derivative on measurement (damps oscillation)
SYNC_KT = 0.5         # Back-calculation anti-windup tracking gain

# Feedforward: PWM% ≈ RPM × FF_GAIN  (empirical from motor_test.py)
# Separate gains for the two encoder-geared motors.
# NOTE: the RR motor may have slightly different friction characteristics,
# producing MORE RPM per PWM% — it therefore needs LESS feedforward per target RPM.
# Setting this equal to or higher than FF_GAIN_METAL caused RR to always
# start with excess PWM, immediately triggering the sync cap and causing rightward drift.
FF_GAIN_METAL = 0.060    # Encoder-geared RL motor: high reduction, lower RPM output
FF_GAIN_PLASTIC = 0.050  # RR motor: slightly lower gain to match behaviour

# Signal conditioning
FILTER_ALPHA = 0.4     # EMA low-pass filter coefficient (0=smooth, 1=raw)

# Slew rate limiters (PWM %/tick at 50Hz → %/second)
SLEW_UP = 6.0          # Max PWM increase per tick (300%/s — fast ramp)
SLEW_DOWN = 100.0      # Max PWM decrease per tick (instant cut for steering)

# Anti-stall minimum PWM floors (per motor)
MIN_PWM_METAL = 8.0     # Encoder-geared RL motor: run at very low power
MIN_PWM_PLASTIC = 18.0  # RR motor: needs this much to not stall

# PWM-to-RPM conversion factor for target calculation
# Derived from: ~300 RPM at ~20% PWM → ratio ≈ 15.0 RPM per PWM%
# This converts commanded PWM% to a target RPM for the PID
PWM_TO_RPM = 15.0

# Stale data threshold — fall back to open-loop if Pico data is older
STALE_THRESHOLD_S = 1.0

# Minimum commanded PWM to activate sync (below this, just pass through)
MIN_ACTIVE_PWM = 3.0

# Convergence band — RPM is "on target" within this tolerance
CONVERGENCE_BAND_RPM = 10.0

# RR kick-start: overcome static friction on idle→active transition.
# A brief PWM burst above the normal anti-stall floor breaks the wheel free;
# after the kick window the PID takes over and syncs RPM to the right side.
RR_KICKSTART_PWM   = 55.0  # PWM % injected during the kick window (must exceed idle stall at ~48%)
RR_KICKSTART_TICKS = 5     # Number of 20 ms ticks (= 100 ms)

# RR stall detection + burst recovery.
# When the wheel is commanded high PWM but encoder shows near-zero RPM for
# several consecutive ticks (gear physically jammed), fire a brief powerful
# burst to break static friction, then let the PID take back over.
RR_STALL_DETECT_PWM   = 30.0   # PWM % above which a near-zero RPM counts as stall
RR_STALL_DETECT_RPM   = 8.0    # RPM below which the wheel is "stalled"
RR_STALL_DETECT_TICKS = 3      # Consecutive stall ticks before triggering burst
RR_STALL_BURST_PWM    = 70.0   # Burst PWM % — strong enough to overcome gear jam
RR_STALL_BURST_TICKS  = 3      # Burst duration (60 ms) — short to avoid overheating

# ──────────────────────────────────────────────────────────────
# Gear-Based RPM Limits
# ──────────────────────────────────────────────────────────────
# Each gear defines (min_rpm, max_rpm).  The PID target is clamped
# to this window so the car never exceeds the gear's intended speed.
# This is critical because the RR motor may spin faster than the
# other motors at the same PWM.  The gear ceiling enforces a maximum
# speed for each gear; the per-wheel PIDs naturally settle at different
# duty cycles to reach the same RPM target given their different efficiencies.

GEAR_RPM_LIMITS = {
    "N": (0, 0),         # Neutral — no drive
    "1": (0, 75),        # Gentle / precision maneuvering
    "2": (75, 120),      # City / moderate cruising
    "3": (120, 200),     # Fast driving
    "4": (200, 300),     # Top gear — full-speed ceiling
    "S": (200, 300),     # Backward-compatible alias for legacy clients
    "R": (0, 120),       # Reverse — moderate
}

# Default RPM limit when gear is unknown
DEFAULT_MAX_RPM = 100


# ──────────────────────────────────────────────────────────────
# Per-Wheel PID State
# ──────────────────────────────────────────────────────────────

class _WheelPIDState:
    """Tracks PID state for a single wheel."""

    __slots__ = (
        'name', 'ff_gain', 'min_pwm',
        'integral', 'filtered_rpm', 'prev_filtered_rpm',
        'prev_output', 'target_rpm', 'actual_rpm',
        'error', 'applied_pwm',
    )

    def __init__(self, name: str, ff_gain: float, min_pwm: float):
        self.name = name
        self.ff_gain = ff_gain
        self.min_pwm = min_pwm
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.filtered_rpm = 0.0
        self.prev_filtered_rpm = 0.0
        self.prev_output = 0.0
        self.target_rpm = 0.0
        self.actual_rpm = 0.0
        self.error = 0.0
        self.applied_pwm = 0.0


# ──────────────────────────────────────────────────────────────
# Main Controller
# ──────────────────────────────────────────────────────────────

class WheelSpeedController:
    """
    Closed-loop wheel speed synchronization.

    Usage:
        ctrl = WheelSpeedController(get_rpm_fn, get_duty_cap_fn)
        # Inside _set_raw_motors:
        rl, rr = ctrl.correct(speed_l, speed_r, duty_cap)
    """

    def __init__(self, get_rpm_fn, get_duty_cap_fn, get_freshness_fn=None):
        """
        Args:
            get_rpm_fn:  callable() -> dict with keys 'rear_left', 'rear_right'
            get_duty_cap_fn: callable() -> float (max safe duty from PowerLimiter)
            get_freshness_fn: callable() -> bool (True if Pico data is fresh)
        """
        self._get_rpm = get_rpm_fn
        self._get_duty_cap = get_duty_cap_fn
        self._get_freshness = get_freshness_fn

        # Per-wheel PID states (RL, RR = encoder-geared rear-drive motors)
        self._rl = _WheelPIDState('rear_left',   FF_GAIN_METAL,   MIN_PWM_METAL)
        self._rr = _WheelPIDState('rear_right',  FF_GAIN_PLASTIC, MIN_PWM_PLASTIC)

        # Controller state
        self._enabled = True
        self._status = "IDLE"       # IDLE | ACTIVE | FALLBACK | OFF
        self._gear = "1"            # Current gear (set by CarSystem)
        self._lock = threading.Lock()

        # Telemetry snapshot (updated each tick, read by UI thread)
        self._telemetry = {}

        # Fallback tracking
        self._fallback_count = 0
        self._active_ticks = 0

        # RR kick-start state
        self._rr_was_idle   = True  # True when right-side commanded speed was below MIN_ACTIVE_PWM
        self._rr_kick_ticks = 0     # Ticks remaining in the current kick-start burst

        # RR stall detection state
        self._rr_stall_ticks = 0    # Consecutive ticks where PWM high but RPM near zero
        self._rr_burst_ticks = 0    # Ticks remaining in the current stall-burst

    # ── Properties ──────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool):
        self._enabled = bool(value)
        if not self._enabled:
            self._status = "OFF"
            self.reset()

    @property
    def status(self) -> str:
        return self._status

    @property
    def gear(self) -> str:
        return self._gear

    @gear.setter
    def gear(self, value: str):
        """Update current gear.  Called from CarSystem before each tick."""
        self._gear = str(value) if value else "1"

    @property
    def gear_max_rpm(self) -> float:
        """Return the current gear's max closed-loop RPM target."""
        return float(GEAR_RPM_LIMITS.get(self._gear, (0, DEFAULT_MAX_RPM))[1])

    @property
    def pwm_to_rpm(self) -> float:
        """Return the PWM->RPM scale used by the legacy sync path."""
        return float(PWM_TO_RPM)

    # ── Core PID Tick ───────────────────────────────────────

    def _pid_tick(self, target_rpm: float, actual_rpm: float,
                  state: _WheelPIDState, duty_cap: float) -> float:
        """
        Run one PID iteration for a single wheel.

        PID + feedforward with back-calculation anti-windup.
        Ported from motor_test.py _pid_step (proven on hardware).

        Args:
            target_rpm: desired wheel RPM
            actual_rpm: measured wheel RPM from encoder
            state: per-wheel PID state object (mutated in place)
            duty_cap: maximum allowed PWM from PowerLimiter

        Returns:
            Corrected PWM duty cycle for this wheel.
        """
        # Store for telemetry
        state.target_rpm = target_rpm
        state.actual_rpm = actual_rpm

        # Low-pass EMA filter on raw RPM to reject encoder noise
        state.prev_filtered_rpm = state.filtered_rpm
        state.filtered_rpm = (FILTER_ALPHA * actual_rpm +
                              (1.0 - FILTER_ALPHA) * state.filtered_rpm)

        # Error (positive = too slow, need more PWM)
        error = target_rpm - state.filtered_rpm
        state.error = error

        # Feedforward: jump-start output near correct PWM
        ff = target_rpm * state.ff_gain

        # Derivative on measurement (not error) — damps oscillation
        # without amplifying setpoint changes.  Negative sign because
        # rising RPM should reduce output.
        d_rpm = state.filtered_rpm - state.prev_filtered_rpm
        d_term = -SYNC_KD * d_rpm

        # Raw (unclamped, un-slewed) PID output
        raw = ff + state.integral + error * SYNC_KP + d_term

        # Hard clamp to valid PWM range
        clamped = max(0.0, min(duty_cap, raw))

        # Asymmetric slew-rate limiter:
        #   Moderate ramp UP  (feedforward already near target)
        #   Fast    ramp DOWN (quick over-speed correction)
        delta = clamped - state.prev_output
        if delta > 0:
            output = state.prev_output + min(delta, SLEW_UP)
        else:
            output = state.prev_output + max(delta, -SLEW_DOWN)
        output = max(0.0, min(duty_cap, output))

        # Anti-stall floor: when the target RPM is above a meaningful
        # threshold, don't let PWM drop below the motor minimum.
        # A very low target (< 5 RPM) means the steering mixer wants
        # this wheel stopped — honour that and let output reach 0.
        if target_rpm > 5.0 and output < state.min_pwm:
            output = state.min_pwm

        # Back-calculation anti-windup:
        # integral += Ki × error + Kt × (actual_applied - raw_desired)
        # When output is clamped/slewed, the tracking term pulls the
        # integral back toward reality, preventing windup.
        state.integral += SYNC_KI * error + SYNC_KT * (output - raw)
        state.integral = max(-duty_cap, min(duty_cap, state.integral))

        state.prev_output = output
        state.applied_pwm = output
        return output

    # ── Main Correction Entry Point ─────────────────────────

    def _correct_impl(
        self,
        speed_l: float,
        speed_r: float,
        target_rpm_l: float,
        target_rpm_r: float,
        duty_cap: float = 63.0,
    ):
        """Shared correction path for PWM-derived and explicit RPM targets."""
        # Not enabled or too-low speed → pass through unchanged
        if not self._enabled:
            self._status = "OFF"
            return speed_l, speed_r

        if (speed_l < MIN_ACTIVE_PWM and speed_r < MIN_ACTIVE_PWM
                and target_rpm_l < 5.0 and target_rpm_r < 5.0):
            self._status = "IDLE"
            # Mark RR as idle so the kick-start fires on the next active tick.
            # Also reset prev_output / integral so the PID starts from a known
            # state rather than resuming at whatever duty was set in the last run.
            self._rr_was_idle = True
            self._rr_stall_ticks = 0
            self._rr_burst_ticks = 0
            self._rr.prev_output = 0.0
            self._rr.integral    = 0.0
            self._rl.prev_output = 0.0
            self._rl.integral    = 0.0
            return speed_l, speed_r

        # Check Pico data freshness
        data_fresh = True
        if self._get_freshness is not None:
            try:
                data_fresh = self._get_freshness()
            except Exception:
                data_fresh = False

        if not data_fresh:
            self._status = "FALLBACK"
            self._fallback_count += 1
            # Open-loop fallback: RR may need a slight boost.
            rl = speed_l           # Encoder-geared motor — no boost needed
            rr = speed_r * 1.15    # RR motor: slight boost to match
            rr = min(duty_cap, rr)
            self._update_telemetry(rl, rr, fallback=True)
            return rl, rr

        # Read live RPM data
        try:
            rpm = self._get_rpm()
        except Exception:
            self._status = "FALLBACK"
            self._fallback_count += 1
            return speed_l, speed_r

        rpm_rl = rpm.get('rear_left', 0.0)
        rpm_rr = rpm.get('rear_right', 0.0)

        # Clamp target RPMs to the current gear's allowed band.
        # This is the key mechanism that enforces gear-based speed
        # limits through closed-loop control — the PID will actively
        # hold the wheels at (or below) the gear's max RPM even if
        # the commanded PWM would produce a higher speed.
        #
        # IMPORTANT: use proportional (ratio-preserving) scaling rather than
        # independent per-side clamping.  Independent clamping silently erases
        # any left/right differential introduced by the gyro heading corrector
        # (which sets speed_l ≠ speed_r to counteract drift).  When both raw
        # targets exceed the gear cap, both would snap to the same max_rpm,
        # making the gyro correction have zero effect at any throttle level
        # where the target exceeds the gear ceiling.
        #
        # Proportional scaling: find the faster side, scale it to max_rpm,
        # apply the same scale factor to the slower side — their ratio is
        # preserved exactly as commanded by the gyro/steering mixer.
        _, max_rpm = GEAR_RPM_LIMITS.get(self._gear, (0, DEFAULT_MAX_RPM))
        if max_rpm > 0:
            fmax = float(max_rpm)
            faster = max(target_rpm_l, target_rpm_r)
            if faster > fmax:
                scale = fmax / faster
                target_rpm_l *= scale
                target_rpm_r *= scale
        else:
            # Neutral or unknown — no drive
            target_rpm_l = 0.0
            target_rpm_r = 0.0

        # ── Hardware-ratio runaway safety abort ─────────────────────────────
        # The RR motor may have slightly different friction or efficiency than
        # the RL motor.  At equal wheel speeds the RR encoder
        # may naturally read somewhat more RPM than the RL encoder because
        # the encoder sits on the motor shaft and individual motor tolerances
        # cause different shaft speeds at the same duty.
        #
        # The previous code capped target_rpm_r to (rpm_rl + 5) whenever
        # rpm_rr exceeded rpm_rl by just 5 RPM.  Telemetry showed this fired
        # IMMEDIATELY during every straight run, forcing:
        #   • RL: 95 % PWM duty (at its physical ceiling, ~163 RPM)
        #   • RR: 18 % PWM floor   (still 242 RPM — impossible target of 168)
        # Because RR at its stall-prevention minimum floor (18 %) still runs
        # ~80 RPM faster than RL at full power (95 %), the cap was setting an
        # unachievable target and the rightward drift accumulated to >20 °.
        #
        # Correct approach: let each PID find its own natural operating point
        # for the given target RPM (the PIDs correctly handle the different
        # motor efficiencies), and rely on the gyro heading corrector in
        # physics_loop for fine balance.  Only abort for a genuine runaway
        # (RR reading more than 250 RPM above RL with both sensors valid),
        # which would indicate a sensor fault or mechanical failure, not
        # normal motor-to-motor variation.
        if rpm_rl > 10.0 and (rpm_rr - rpm_rl) > 250.0:
            target_rpm_r = min(target_rpm_r, rpm_rl + 250.0)

        # RR kick-start: detect idle→active transition on the right side.
        # When the right side was idle (speed_r ≈ 0) and is now commanded to
        # move, arm a short PWM burst to overcome static friction.
        if speed_r >= MIN_ACTIVE_PWM and self._rr_was_idle and target_rpm_r > 5.0:
            self._rr_kick_ticks = RR_KICKSTART_TICKS
            self._rr_stall_ticks = 0  # fresh start — don't immediately stall-detect
        self._rr_was_idle = speed_r < MIN_ACTIVE_PWM

        # Run per-wheel PID controllers
        rl_pwm = self._pid_tick(target_rpm_l, rpm_rl, self._rl, duty_cap)
        rr_pwm = self._pid_tick(target_rpm_r, rpm_rr, self._rr, duty_cap)

        # Apply RR kick-start burst if active (overrides PID output upward only)
        if self._rr_kick_ticks > 0:
            rr_pwm = max(rr_pwm, min(RR_KICKSTART_PWM, duty_cap))
            self._rr_kick_ticks -= 1

        # RR stall detection + burst recovery.
        # If RR is commanded to move but its own encoder still reads near-zero,
        # force a strong kick.
        rr_should_spin = target_rpm_r > 5.0 and speed_r >= MIN_ACTIVE_PWM
        rr_not_spinning = rpm_rr < RR_STALL_DETECT_RPM

        if rr_should_spin and rr_not_spinning:
            self._rr_stall_ticks += 1
            if self._rr_stall_ticks >= RR_STALL_DETECT_TICKS:
                self._rr_burst_ticks = max(self._rr_burst_ticks, RR_STALL_BURST_TICKS)
                self._rr_stall_ticks = 0
        else:
            self._rr_stall_ticks = 0

        if self._rr_burst_ticks > 0:
            rr_pwm = duty_cap
            self._rr_burst_ticks -= 1
            # Also zero the PID state so it ramps cleanly after the burst
            self._rr.prev_output = rr_pwm
            self._rr.integral = 0.0

        # Hard override: when one side is commanded to 0 by the steering
        # mixer, bypass the PID slew limiter and force immediate 0 PWM.
        # Without this, the slew-down rate (4%/tick) causes a ~160ms lag
        # before the inner wheels actually stop, ruining the turn.
        if speed_l < MIN_ACTIVE_PWM and target_rpm_l < 5.0:
            rl_pwm = 0.0
            self._rl.prev_output = 0.0
            self._rl.integral = 0.0
        if speed_r < MIN_ACTIVE_PWM and target_rpm_r < 5.0:
            rr_pwm = 0.0
            self._rr.prev_output = 0.0
            self._rr.integral = 0.0

        self._status = "ACTIVE"
        self._active_ticks += 1

        self._update_telemetry(rl_pwm, rr_pwm)
        return rl_pwm, rr_pwm

    def correct(self, speed_l: float, speed_r: float, duty_cap: float = 63.0):
        """
        Apply closed-loop wheel speed corrections from left/right PWM inputs.

        Called from CarSystem._set_raw_motors() every tick (~50Hz). The PWM
        inputs are converted into side RPM targets, then fed through the
        shared per-wheel closed-loop correction path.
        """
        target_rpm_l = speed_l * PWM_TO_RPM
        target_rpm_r = speed_r * PWM_TO_RPM
        return self._correct_impl(speed_l, speed_r, target_rpm_l, target_rpm_r, duty_cap)

    def correct_target_rpms(
        self,
        target_rpm_l: float,
        target_rpm_r: float,
        speed_l_hint: float = 0.0,
        speed_r_hint: float = 0.0,
        duty_cap: float = 63.0,
    ):
        """Apply closed-loop wheel speed corrections from explicit side RPM targets."""
        return self._correct_impl(
            float(speed_l_hint),
            float(speed_r_hint),
            float(target_rpm_l),
            float(target_rpm_r),
            duty_cap,
        )

    # ── Reset ───────────────────────────────────────────────

    def reset(self):
        """Zero all PID state.  Call on stop, brake, direction change."""
        self._rl.reset()
        self._rr.reset()
        self._rr_was_idle    = True
        self._rr_kick_ticks  = 0
        self._rr_stall_ticks = 0
        self._rr_burst_ticks = 0
        # Don't change _enabled or _status here

    # ── Telemetry ───────────────────────────────────────────

    def _update_telemetry(self, rl_pwm, rr_pwm, fallback=False):
        """Snapshot per-wheel data for the web UI (thread-safe read)."""
        _, gear_max_rpm = GEAR_RPM_LIMITS.get(self._gear, (0, DEFAULT_MAX_RPM))
        self._telemetry = {
            'status': self._status,
            'active_ticks': self._active_ticks,
            'fallback_count': self._fallback_count,
            'gear': self._gear,
            'gear_max_rpm': gear_max_rpm,
            'wheels': {
                'rl': {
                    'target_rpm': round(self._rl.target_rpm, 1),
                    'actual_rpm': round(self._rl.actual_rpm, 1),
                    'applied_pwm': round(rl_pwm, 1),
                    'error': round(self._rl.error, 1),
                    'has_encoder': True,
                    'on_target': abs(self._rl.error) <= CONVERGENCE_BAND_RPM,
                },
                'rr': {
                    'target_rpm': round(self._rr.target_rpm, 1),
                    'actual_rpm': round(self._rr.actual_rpm, 1),
                    'applied_pwm': round(rr_pwm, 1),
                    'error': round(self._rr.error, 1),
                    'has_encoder': True,
                    'on_target': abs(self._rr.error) <= CONVERGENCE_BAND_RPM,
                },
            },
        }

    def get_telemetry(self) -> dict:
        """Return latest sync telemetry snapshot for the web UI."""
        return self._telemetry.copy() if self._telemetry else {
            'status': self._status,
            'active_ticks': 0,
            'fallback_count': 0,
            'wheels': {},
        }

    def __repr__(self):
        return (f"WheelSpeedController(status={self._status}, "
                f"enabled={self._enabled}, ticks={self._active_ticks})")
