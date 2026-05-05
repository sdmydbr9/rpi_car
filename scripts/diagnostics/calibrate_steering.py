#!/usr/bin/env python3
"""
calibrate_steering.py - Interactive steering servo calibration tool.

Test the steering servo independently by communicating directly with the Pico.
Allows fine-tuning of steering response and verifying mechanical limits.

Controls:
  L - Move steering left in small steps
  R - Move steering right in small steps
  C - Snap steering to center
  S - Run Sweep Test Sequence
  Q - Exit and snap steering to center

Steering range: -50° (full left) to +50° (full right), center at 0°.
Step size: 5° per keypress (adjustable).
"""

import sys
import os
import time
import threading
from typing import Optional

# Add core directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'core'))

try:
    from steering_calibration import (
        DEFAULT_STEERING_CALIBRATION,
        steering_angle_to_pw,
    )
    from pico_sensor_reader import init_pico_reader, send_lr_pwm
except ImportError as e:
    print(f"❌ Failed to import required modules: {e}")
    print("Make sure this script is run from the scripts directory.")
    sys.exit(1)


class SteeringCalibrator:
    """Interactive steering servo calibration tool."""

    # Configuration
    STEP_SIZE = 5  # Degrees per keypress
    MIN_ANGLE = -50
    MAX_ANGLE = 50
    CENTER_ANGLE = 0

    def __init__(self, port: str = '/dev/ttyS0'):
        """Initialize the steering calibrator.
        
        Args:
            port: Serial port for Pico communication (default: /dev/ttyS0)
        """
        self.port = port
        self.current_angle = self.CENTER_ANGLE
        self.running = False
        self.pico_reader = None
        # Use explicit pulse width calibration
        self.calibration = {
            "left_pw": 940,      # Full left
            "center_pw": 1440,   # Center
            "right_pw": 2150,    # Full right
        }
        
        # Threading for non-blocking input
        self._input_thread = None
        self._stop_event = threading.Event()

    def _angle_to_pw(self, angle: float) -> int:
        """Convert steering angle to servo pulse width."""
        return steering_angle_to_pw(angle, self.calibration)

    def _send_steering(self, angle: float) -> None:
        """Send steering command to Pico."""
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle))
        pw = self._angle_to_pw(angle)
        send_lr_pwm(0, 0, pw, forward=True)

    def _display_status(self) -> None:
        """Display current steering status."""
        pw = self._angle_to_pw(self.current_angle)
        bar_width = 40
        # Map angle (-50 to +50) to bar position (0 to 39)
        bar_pos = int((self.current_angle - self.MIN_ANGLE) / 
                     (self.MAX_ANGLE - self.MIN_ANGLE) * (bar_width - 1))
        bar = ['─'] * bar_width
        bar[bar_pos] = '●'
        bar_str = ''.join(bar)
        
        print(f"\r📍 Steering: {self.current_angle:+3d}° [{bar_str}] | "
              f"PW: {pw}µs | "
              f"Left({self.MIN_ANGLE}°)─Center(0°)─Right({self.MAX_ANGLE}°)",
              end='', flush=True)
        
        print(f"\r📍 Steering: {self.current_angle:+3d}° [{bar_str}] | "
              f"PW: {pw}µs | "
              f"Left({self.MIN_ANGLE}°)─Center(0°)─Right({self.MAX_ANGLE}°)",
              end='', flush=True)

    def _run_sweep_test(self) -> None:
        """Executes a test sequence across the limits to verify Pico slew limiting."""
        print("\n\n--- Running Sweep Test ---")
        for angle in [self.MIN_ANGLE, self.MAX_ANGLE, self.MIN_ANGLE, self.CENTER_ANGLE]:
            self.current_angle = angle
            self._send_steering(self.current_angle)
            self._display_status()
            # Give the Pico time to execute the physical ramp before sending the next target
            time.sleep(0.5) 
        print("\nSweep test complete.")
        self._display_status()
        print() # New line for clean formatting

    def _input_thread_func(self) -> None:
        """Read keyboard input in a background thread."""
        try:
            import tty
            import termios
        except ImportError:
            print("❌ termios module not available (Unix-only). Using fallback input.")
            self._blocking_input_fallback()
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(fd)
            
            while not self._stop_event.is_set():
                try:
                    ch = sys.stdin.read(1).upper()
                    
                    if ch == 'L':
                        # Move left
                        new_angle = self.current_angle - self.STEP_SIZE
                        self.current_angle = max(self.MIN_ANGLE, new_angle)
                        self._send_steering(self.current_angle)
                        self._display_status()
                        
                    elif ch == 'R':
                        # Move right
                        new_angle = self.current_angle + self.STEP_SIZE
                        self.current_angle = min(self.MAX_ANGLE, new_angle)
                        self._send_steering(self.current_angle)
                        self._display_status()
                        
                    elif ch == 'C':
                        # Center
                        self.current_angle = self.CENTER_ANGLE
                        self._send_steering(self.current_angle)
                        self._display_status()
                        print()  # New line after center command
                        
                    elif ch == 'S':
                        # Sweep Test
                        self._run_sweep_test()

                    elif ch == 'Q':
                        # Quit
                        break
                    
                except KeyboardInterrupt:
                    break
                    
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _blocking_input_fallback(self) -> None:
        """Fallback input method for systems without termios."""
        print("\n⚠️ Using standard input (non-raw mode). Press Enter after each command.\n")
        
        while not self._stop_event.is_set():
            try:
                user_input = input("Command (L/R/C/S/Q): ").strip().upper()
                
                if user_input == 'L':
                    new_angle = self.current_angle - self.STEP_SIZE
                    self.current_angle = max(self.MIN_ANGLE, new_angle)
                    self._send_steering(self.current_angle)
                    self._display_status()
                    print()  # New line
                    
                elif user_input == 'R':
                    new_angle = self.current_angle + self.STEP_SIZE
                    self.current_angle = min(self.MAX_ANGLE, new_angle)
                    self._send_steering(self.current_angle)
                    self._display_status()
                    print()  # New line
                    
                elif user_input == 'C':
                    self.current_angle = self.CENTER_ANGLE
                    self._send_steering(self.current_angle)
                    self._display_status()
                    print()  # New line
                    
                elif user_input == 'S':
                    self._run_sweep_test()

                elif user_input == 'Q':
                    break
                else:
                    print("Invalid command. Use L, R, C, S, or Q.")
                    
            except KeyboardInterrupt:
                break

    def initialize(self) -> bool:
        """Initialize Pico connection.
        
        Returns:
            True if successful, False otherwise.
        """
        try:
            print("🔌 Initializing Pico sensor reader...")
            self.pico_reader = init_pico_reader(self.port)
            time.sleep(0.5)  # Give it a moment to connect
            
            if not self.pico_reader:
                print("❌ Failed to initialize Pico reader")
                return False
            
            print("✅ Pico connection established")
            return True
            
        except Exception as e:
            print(f"❌ Error initializing Pico: {e}")
            return False

    def run(self) -> None:
        """Run the interactive calibration session."""
        if not self.initialize():
            sys.exit(1)
        
        self.running = True
        
        try:
            # Center the steering on startup
            print("🎛️  Setting steering to center...")
            self._send_steering(self.CENTER_ANGLE)
            time.sleep(0.2)
            
            print("\n" + "="*80)
            print("STEERING CALIBRATOR - Interactive Mode")
            print("="*80)
            print(f"\nStep size: {self.STEP_SIZE}°")
            print(f"Range: {self.MIN_ANGLE}° to {self.MAX_ANGLE}°")
            print("\nControls:")
            print("  L - Move left (current - 5°)")
            print("  R - Move right (current + 5°)")
            print("  C - Snap to center")
            print("  S - Run Sweep Test Sequence")
            print("  Q - Exit and center")
            print("\n" + "="*80 + "\n")
            
            # Display initial status
            self._display_status()
            
            # Start input thread
            self._stop_event.clear()
            self._input_thread = threading.Thread(
                target=self._input_thread_func,
                daemon=False
            )
            self._input_thread.start()
            
            # Wait for input thread to finish
            self._input_thread.join()
            
            # Clean shutdown - center the servo
            print("\n\n🎛️  Centering servo on exit...")
            self.current_angle = self.CENTER_ANGLE
            self._send_steering(self.current_angle)
            time.sleep(0.2)
            
            print("✅ Steering calibration complete")
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted - centering servo...")
            self.current_angle = self.CENTER_ANGLE
            self._send_steering(self.current_angle)
            time.sleep(0.2)
            print("✅ Steering centered")
            
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Clean up resources."""
        self._stop_event.set()
        
        if self._input_thread and self._input_thread.is_alive():
            self._input_thread.join(timeout=1.0)
        
        if self.pico_reader:
            try:
                self.pico_reader.close()
                print("✅ Pico connection closed")
            except Exception as e:
                print(f"⚠️  Error closing Pico connection: {e}")
        
        self.running = False


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Interactive steering servo calibration tool'
    )
    parser.add_argument(
        '--port',
        default='/dev/ttyS0',
        help='Serial port for Pico communication (default: /dev/ttyS0)'
    )
    parser.add_argument(
        '--step',
        type=int,
        default=5,
        help='Step size in degrees (default: 5)'
    )
    
    args = parser.parse_args()
    
    calibrator = SteeringCalibrator(port=args.port)
    calibrator.STEP_SIZE = args.step
    calibrator.run()


if __name__ == '__main__':
    main()
