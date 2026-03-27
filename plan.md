# Line-Follow Controller Optimization Plan

## Summary
- Create and keep this file as the implementation record for the line-follow controller optimization pass.
- Improve the `follow_line.py` detection, steering, recovery, speed, and telemetry paths.
- Extend the motor stack so follow-line can command explicit left/right target RPMs through `wheel_sync.py` instead of relying only on PWM-derived targets.

## Key Changes
- Detection pipeline:
  - Shrink the horizontal ROI around `_locked_cx` by `±150px` once the line is locked.
  - Run the full detector on a `0.5x` resized ROI and remap centroids, contours, and masks back to frame coordinates.
  - Track the last 5 strong template matches and switch locked high-confidence frames to a lightweight centroid-only scan with same-frame fallback to the full detector.
  - Keep centroid math in float until draw/output time.
  - Use `3x3` morphology while locked and `5x5` while searching or recovering.
  - Reuse a single contour-mask scratch buffer instead of allocating a new mask per candidate.
- Curve geometry and steering:
  - Use derivative-on-measurement for the steering PID.
  - Schedule `kp` down at higher measured speeds.
  - Freeze integral accumulation for large line errors.
  - Raise lookahead blending on higher curvature and keep the disagreement fade.
  - Use weighted quadratic polyfit, predict a short-horizon centerline x, and feed that into a feedforward steering term.
  - Suppress absolute-heading correction while active cornering is underway and let yaw-rate control dominate.
- Speed, recovery, and cadence:
  - Slow down from the stronger of steer magnitude and normalized lookahead error.
  - Add a recovery brake/hold/re-accelerate sequence.
  - Above `0.30 m/s`, detect every frame but reuse the prior command on alternating control updates.
- Sensor fusion and motor stack:
  - Estimate gyro bias only while effectively stationary and subtract it from yaw/heading updates.
  - Blend encoder yaw rate only when both sides are above `ENCODER_MIN_RPM`.
  - Add explicit side target-RPM control to `CarSystem` and `WheelSpeedController`, then drive follow-line through that path when encoders are enabled.

## Public Interfaces
- Extend `/api/status` and CSV telemetry with:
  - `detection_mode`
  - `template_confidence`
  - `predicted_cx`
  - `feedforward_steer`
  - `gyro_bias`
  - `recovery_phase`
  - `control_skip`
  - `target_rpm_l`
  - `target_rpm_r`
- Keep the existing raw motor command path available for other callers.

## Validation Checklist
- Run Python bytecode compilation on the touched modules.
- Run unit tests that cover:
  - Curve prediction output from the weighted spine fit.
  - Explicit target-RPM wheel-sync control and proportional gear-cap limiting.
- Hardware follow-up:
  - Confirm fast-path detection stays locked on straights and falls back cleanly when needed.
  - Confirm brake-then-recover behavior on tight corners.
  - Confirm the target-RPM path holds left/right speed more consistently than the old PWM-only path.
  - Confirm gyro bias only adapts while stationary.

## Tuning Follow-Ups
- Re-tune `FEEDFORWARD_KP` on hardware if the rover turns in too early or too aggressively.
- Re-tune the high-speed control skip threshold if the steering starts to lag on fast S-curves.
- Revisit the recovery brake/hold length if reacquisition becomes too abrupt or too soft.
