"""lgpio_encoder.py — Quadrature encoder reader using lgpio.

Uses the kernel character device interface (/dev/gpiochipN) for reliable
GPIO edge detection on modern kernels (6.x+).  pigpio's DMA-based sampling
is broken on kernel 6.12+ for the Pi 4.

Provides a drop-in replacement for gpiozero.RotaryEncoder with a `.steps`
attribute that tracks cumulative encoder ticks.

Usage:
    import lgpio
    chip = lgpio.gpiochip_open(0)
    enc = LgpioEncoder(chip, pin_a=27, pin_b=17)
    # ... read enc.steps in a loop ...
    enc.cancel()
    lgpio.gpiochip_close(chip)
"""

import lgpio


class LgpioEncoder:
    """Robust state-machine quadrature encoder using lgpio callbacks.

    Counts every edge transition (4× per full quadrature cycle).
    To get the same count as gpiozero RotaryEncoder, divide .steps by 4
    or multiply PPR by 4 when computing RPM.
    """

    # Transition table: (old_state << 2 | new_state) → step direction
    # Filters out bounce and invalid transitions automatically.
    TRANSITIONS = [
         0, -1,  1,  0,  # old=00
         1,  0,  0, -1,  # old=01
        -1,  0,  0,  1,  # old=10
         0,  1, -1,  0,  # old=11
    ]

    def __init__(self, chip_handle, pin_a, pin_b):
        self.h = chip_handle
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.steps = 0

        # Claim pins as alert inputs with pull-ups for edge detection
        lgpio.gpio_claim_alert(self.h, pin_a, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_alert(self.h, pin_b, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)

        # Initialize local state
        self.lev_a = lgpio.gpio_read(self.h, pin_a)
        self.lev_b = lgpio.gpio_read(self.h, pin_b)
        self.old_state = (self.lev_a << 1) | self.lev_b

        # Attach edge callbacks
        self._cb_a = lgpio.callback(self.h, pin_a, lgpio.BOTH_EDGES, self._pulse)
        self._cb_b = lgpio.callback(self.h, pin_b, lgpio.BOTH_EDGES, self._pulse)

    def _pulse(self, chip, gpio, level, tick):
        if gpio == self.pin_a:
            self.lev_a = level
        else:
            self.lev_b = level

        new_state = (self.lev_a << 1) | self.lev_b
        index = (self.old_state << 2) | new_state

        self.steps += self.TRANSITIONS[index]
        self.old_state = new_state

    def cancel(self):
        """Clean up callbacks to free system resources."""
        self._cb_a.cancel()
        self._cb_b.cancel()
