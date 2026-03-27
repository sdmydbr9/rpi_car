from machine import Pin
import time

class MotorEncoder:
    def __init__(self, pin_a_num, pin_b_num):
        # Set up the pins as inputs with pull-up resistors
        self.pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)
        
        # This variable tracks the total steps
        self.position = 0
        
        # Attach an interrupt to Phase A. It triggers every time the signal goes HIGH or LOW.
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._isr)

    def _isr(self, pin):
        # Interrupt Service Routine (ISR)
        # We read both pins to figure out the direction of rotation
        a_val = self.pin_a.value()
        b_val = self.pin_b.value()
        
        # If A and B match, count up. If they differ, count down.
        if a_val == b_val:
            self.position += 1
        else:
            self.position -= 1

    def get_count(self):
        return self.position

    def reset_count(self):
        self.position = 0

# --- Main Program Setup ---

# Initialize Left and Right Encoders using the GP pins we mapped out
left_encoder = MotorEncoder(3, 2)   # GP2 = Phase A, GP3 = Phase B
right_encoder = MotorEncoder(4, 5)  # GP4 = Phase A, GP5 = Phase B

print("Starting encoder test. Spin the wheels by hand to see the counts change!")

try:
    # Main loop to display the data
    while True:
        left_count = left_encoder.get_count()
        right_count = right_encoder.get_count()
        
        # Print the raw counts to the console
        print(f"Left Position: {left_count} | Right Position: {right_count}")
        
        # Update the screen twice a second
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nTest stopped.")

