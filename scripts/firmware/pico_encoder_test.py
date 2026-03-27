# pico_encoder_test.py — Standalone PIO encoder + RPM test
#
# Upload to Pico, run via Thonny. Spin wheels by hand to verify
# encoder steps and RPM. Prints to USB serial (Thonny console).
#
# Pin wiring (same as pico_sensor_bridge.py):
#   Left:  Phase A = GP3, Phase B = GP2
#   Right: Phase A = GP4, Phase B = GP5

from machine import Pin
import rp2
import time

print("\n" + "="*50)
print("  PIO ENCODER + RPM TEST")
print("  Spin wheels by hand to verify readings")
print("="*50 + "\n")

# =====================================================
# PIO ENCODER (verified working method)
# =====================================================

@rp2.asm_pio()
def pio_encoder():
    wrap_target()
    wait(1, pin, 0)
    jmp(pin, "count_up")

    jmp(x_dec, "wait_low")

    label("count_up")
    mov(x, invert(x))
    jmp(x_dec, "re_invert")
    label("re_invert")
    mov(x, invert(x))

    label("wait_low")
    wait(0, pin, 0)
    wrap()


class PIOMotorEncoder:
    def __init__(self, sm_id, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self.sm = rp2.StateMachine(
            sm_id, pio_encoder, freq=10_000_000,
            in_base=self.pin_a, jmp_pin=self.pin_b
        )
        self.sm.exec("set(x, 0)")
        self.sm.active(1)

        self.zero_offset = 0

    def get_raw_count(self):
        self.sm.exec("in_(x, 32)")
        self.sm.exec("push()")
        val = self.sm.get()
        if val > (1 << 31):
            val -= (1 << 32)
        return val

    def get_position(self):
        return self.get_raw_count() - self.zero_offset

    def reset_position(self):
        self.zero_offset = self.get_raw_count()


# =====================================================
# INIT ENCODERS (same pins as bridge)
# =====================================================
enc_left  = PIOMotorEncoder(sm_id=0, pin_a=3, pin_b=2)
enc_right = PIOMotorEncoder(sm_id=1, pin_a=4, pin_b=5)
print("✅ Left  encoder: GP3(A) / GP2(B)  — SM0")
print("✅ Right encoder: GP4(A) / GP5(B)  — SM1")

# ── RPM config (same as bridge) ──
PPR = 330.0
ALPHA_RPM = 0.3

# ── Pin state debug ──
print(f"\n📌 Initial pin states:")
print(f"   Left  A(GP3)={Pin(3, Pin.IN, Pin.PULL_UP).value()}  B(GP2)={Pin(2, Pin.IN, Pin.PULL_UP).value()}")
print(f"   Right A(GP4)={Pin(4, Pin.IN, Pin.PULL_UP).value()}  B(GP5)={Pin(5, Pin.IN, Pin.PULL_UP).value()}")

print(f"\n🔧 Config: PPR={PPR}, EMA α={ALPHA_RPM}")
print("   Spin a wheel — you should see position and RPM change.\n")

# =====================================================
# MAIN LOOP — RPM at ~10 Hz, print at ~5 Hz
# =====================================================
prev_raw_l = enc_left.get_raw_count()
prev_raw_r = enc_right.get_raw_count()
prev_rpm_time = time.ticks_ms()
rpm_filt_l = 0.0
rpm_filt_r = 0.0
rpm_l = 0.0
rpm_r = 0.0
print_tick = 0

try:
    while True:
        t0 = time.ticks_ms()
        dt_ms = time.ticks_diff(t0, prev_rpm_time)

        if dt_ms >= 100:
            prev_rpm_time = t0
            raw_l = enc_left.get_raw_count()
            raw_r = enc_right.get_raw_count()
            dl = raw_l - prev_raw_l
            dr = raw_r - prev_raw_r
            prev_raw_l = raw_l
            prev_raw_r = raw_r

            rpm_l = (dl / PPR) * (60000.0 / dt_ms)
            rpm_r = (dr / PPR) * (60000.0 / dt_ms)
            rpm_filt_l = ALPHA_RPM * rpm_l + (1.0 - ALPHA_RPM) * rpm_filt_l
            rpm_filt_r = ALPHA_RPM * rpm_r + (1.0 - ALPHA_RPM) * rpm_filt_r

            l_pos = enc_left.get_position()
            r_pos = enc_right.get_position()

            print(f"L pos:{l_pos:6d}  RPM:{rpm_filt_l:6.1f} (raw:{rpm_l:6.1f})  |  "
                  f"R pos:{r_pos:6d}  RPM:{rpm_filt_r:6.1f} (raw:{rpm_r:6.1f})")

        time.sleep_ms(10)

except KeyboardInterrupt:
    print(f"\n🛑 Stopped. Final pos — L:{enc_left.get_position()}  R:{enc_right.get_position()}")
