#!/usr/bin/env python3
"""
Test: Verify the autopilot enable/disable bug is fixed.

Bug description:
  1. When autopilot starts, UI shows CRUISING but acceleration=0% and car
     doesn't move.
  2. When stop button is pressed, the motor moves for 1-2 seconds before
     stopping.

Root causes:
  A. Duplicate PWM objects on ENA/ENB: main.py created module-level
     pwm_a/pwm_b *and* CarSystem created its own pair on the same pins.
     The two software-PWM threads interfered, so CarSystem's writes
     (from the autopilot) were cancelled by the stale 0% module-level PWMs.
  B. Race condition in on_autonomous_disable: autonomous_mode was set to
     False BEFORE motors were stopped.  physics_loop saw manual-mode with
     a stale current_pwm (60) and coasted the car for ~2 seconds.

Fix:
  A. After CarSystem init, stop the old module-level PWMs and point
     pwm_a / pwm_b to car_system's instances.
  B. Stop motors and zero current_pwm BEFORE clearing autonomous_mode.

This test connects via SocketIO and checks:
  1. After enabling autopilot, autonomous_target_speed > 0 within 2 seconds.
  2. After disabling autopilot, current_pwm == 0 immediately (no coasting).
"""

import socketio
import time
import sys
import threading

sio = socketio.Client(reconnection=True, reconnection_attempts=3, reconnection_delay=1)

# Telemetry collection
telemetry_log = []
telemetry_lock = threading.Lock()

# Test results
test_results = {}


@sio.event
def connect():
    print("✅ Connected to server")


@sio.event
def disconnect():
    print("❌ Disconnected from server")


@sio.on('telemetry_update')
def on_telemetry(data):
    with telemetry_lock:
        telemetry_log.append({
            'time': time.time(),
            'autonomous_mode': data.get('autonomous_mode', False),
            'autonomous_state': data.get('autonomous_state', ''),
            'autonomous_target_speed': data.get('autonomous_target_speed', 0),
            'current_pwm': data.get('current_pwm', 0),
            'gear': data.get('gear', 'N'),
        })


@sio.on('autonomous_response')
def on_autonomous_response(data):
    print(f"   → Autonomous response: {data}")


def get_latest_telemetry():
    """Return the most recent telemetry entry (or None)."""
    with telemetry_lock:
        return telemetry_log[-1].copy() if telemetry_log else None


def wait_for_condition(predicate, timeout=3.0, poll=0.1, desc="condition"):
    """Poll telemetry until *predicate(entry)* is True or timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        entry = get_latest_telemetry()
        if entry and predicate(entry):
            return entry
        time.sleep(poll)
    return get_latest_telemetry()  # return last even if failed


def run_tests():
    print("\n" + "=" * 70)
    print("🧪 AUTOPILOT BUG-FIX VERIFICATION TEST")
    print("=" * 70)
    all_passed = True

    # ------------------------------------------------------------------
    # TEST 1 – Autopilot produces non-zero speed while CRUISING
    # ------------------------------------------------------------------
    print("\n[TEST 1] Autopilot enable → speed should be > 0 within 2s")
    print("-" * 70)

    # Clear log
    with telemetry_lock:
        telemetry_log.clear()

    # Enable autopilot
    sio.emit('autonomous_enable', {})
    time.sleep(0.3)  # let the server process the event

    entry = wait_for_condition(
        lambda e: e['autonomous_mode'] and e['autonomous_target_speed'] > 0,
        timeout=3.0,
        desc="autopilot speed > 0",
    )

    if entry and entry['autonomous_target_speed'] > 0:
        print(f"   ✅ PASS — autonomous_target_speed = {entry['autonomous_target_speed']:.1f}%")
        print(f"            state = {entry['autonomous_state']}, mode = {entry['autonomous_mode']}")
        test_results['enable_speed'] = 'PASS'
    else:
        speed = entry['autonomous_target_speed'] if entry else 'N/A'
        print(f"   ❌ FAIL — autonomous_target_speed = {speed}  (expected > 0)")
        test_results['enable_speed'] = 'FAIL'
        all_passed = False

    # ------------------------------------------------------------------
    # TEST 2 – Disabling autopilot zeroes speed immediately (no coasting)
    # ------------------------------------------------------------------
    print("\n[TEST 2] Autopilot disable → current_pwm should be 0 immediately")
    print("-" * 70)

    # Clear log
    with telemetry_lock:
        telemetry_log.clear()

    # Disable autopilot
    sio.emit('autonomous_disable', {})
    time.sleep(0.5)  # small wait for the server to process + next telemetry tick

    entry = get_latest_telemetry()

    if entry and entry['current_pwm'] == 0:
        print(f"   ✅ PASS — current_pwm = {entry['current_pwm']:.1f}%  (immediate stop)")
        print(f"            gear = {entry['gear']}, mode = {entry['autonomous_mode']}")
        test_results['disable_coast'] = 'PASS'
    else:
        pwm = entry['current_pwm'] if entry else 'N/A'
        print(f"   ❌ FAIL — current_pwm = {pwm}  (expected 0, car is coasting!)")
        test_results['disable_coast'] = 'FAIL'
        all_passed = False

    # ------------------------------------------------------------------
    # TEST 3 – After disable, no residual coasting over 1 second
    # ------------------------------------------------------------------
    print("\n[TEST 3] No residual motor activity 1s after disable")
    print("-" * 70)

    time.sleep(1.0)
    with telemetry_lock:
        recent = [e for e in telemetry_log if e['current_pwm'] > 0]

    if not recent:
        print(f"   ✅ PASS — No non-zero current_pwm readings after disable")
        test_results['no_coast'] = 'PASS'
    else:
        max_pwm = max(e['current_pwm'] for e in recent)
        print(f"   ❌ FAIL — Found {len(recent)} non-zero readings (max {max_pwm:.1f}%)")
        test_results['no_coast'] = 'FAIL'
        all_passed = False

    # ------------------------------------------------------------------
    # SUMMARY
    # ------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("📊 TEST SUMMARY")
    print("=" * 70)
    for name, result in test_results.items():
        icon = "✅" if result == 'PASS' else "❌"
        print(f"   {icon}  {name}: {result}")

    if all_passed:
        print("\n🎉 ALL TESTS PASSED — Autopilot bug is FIXED!")
    else:
        print("\n⚠️  SOME TESTS FAILED — Bug may still be present.")
    print("=" * 70)

    return all_passed


# ── Main ──────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        print("🔌 Connecting to server at localhost:5000 ...")
        sio.connect('http://localhost:5000', wait_timeout=5)
        time.sleep(1)  # let telemetry start flowing

        passed = run_tests()

        sio.disconnect()
        sys.exit(0 if passed else 1)
    except Exception as e:
        print(f"❌ Connection error: {e}")
        sys.exit(1)
