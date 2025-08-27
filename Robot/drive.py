"""
drive.py

This module defines the Motor and DriveSystem classes for controlling the robot's movement.
Each Motor instance manages speed and direction via GPIO-based PWM control, while
DriveSystem coordinates both motors using predefined movement styles.

Includes:
- Motor class for low-level control of individual motors using PWM
- DriveSystem class for high-level motion using styles (forward, turns, spins, etc.)
- Functions to apply custom PWM values and stop all motion
"""

import pigpio
import sys
import time
import constants  # Imports styles and pin assignments
from constants import MovementStyle


class Motor:
    """
    Controls a single motor using two GPIO pins.
    """

    def __init__(self, pi, pin_a, pin_b):
        """
        Initializes the motor with two output pins and configures PWM settings.
        """
        self.pi = pi
        self.pin_a = pin_a
        self.pin_b = pin_b

        # Configure pins
        self.pi.set_mode(pin_a, pigpio.OUTPUT)
        self.pi.set_mode(pin_b, pigpio.OUTPUT)
        self.pi.set_PWM_range(pin_a, 255)
        self.pi.set_PWM_range(pin_b, 255)
        self.pi.set_PWM_frequency(pin_a, 1000)
        self.pi.set_PWM_frequency(pin_b, 1000)

    def set_level(self, level):
        """
        Sets motor speed and direction using the actual PWM range.
        """
        pwm_range = self.pi.get_PWM_range(self.pin_a)
        speed = int(abs(level) * pwm_range)

        if level > 0:
            self.pi.set_PWM_dutycycle(self.pin_a, speed)
            self.pi.set_PWM_dutycycle(self.pin_b, 0)
        elif level < 0:
            self.pi.set_PWM_dutycycle(self.pin_a, 0)
            self.pi.set_PWM_dutycycle(self.pin_b, speed)
        else:
            self.pi.set_PWM_dutycycle(self.pin_a, 0)
            self.pi.set_PWM_dutycycle(self.pin_b, 0)


    def stop(self):
        """Stops the motor."""
        self.set_level(0)


class DriveSystem:
    """
    Controls left and right motors based on a movement style.
    """

    def __init__(self, io, pin1 = constants.PIN1, pin2 = constants.PIN2, pin3 = constants.PIN3, pin4 = constants.PIN4):
        """
        Initializes the DriveSystem with left and right motors.
        """
        self.io = io 
        self.motor_r = Motor(io, pin1, pin2)
        self.motor_l = Motor(io, pin3, pin4)

    def drive(self, style, backwards=False):
        """
        Drive the robot based on predefined styles from constants.
        """
        mult = -1 if backwards else 1
        if style in constants.MovementStyle:
            motor_r_level = style.value["motor_r"]
            motor_l_level = style.value["motor_l"]
            
        self.motor_r.set_level(mult * style.value["motor_r"])
        self.motor_l.set_level(mult * style.value["motor_l"])
        
    def pwm(self, pwm_left, pwm_right):
        """
        Directly set PWM levels to the left and right motors..
        """
        self.motor_l.set_level(pwm_left)
        self.motor_r.set_level(pwm_right)


    def stop(self):
        """Stops both motors."""
        self.motor_r.stop()
        self.motor_l.stop()


if __name__ == "__main__":
    """
    If run directly, this script allows for manual testing of motor movements
    using keyboard input. Requires a working pigpio daemon.
    """
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon.")
        sys.exit(0)

    robot = DriveSystem(io, constants.PIN1, constants.PIN2, constants.PIN3, constants.PIN4)

    print("Manual Test Mode. Type a movement command ('list' to see options, 'exit' to quit).")

    try:
        while True:
            command = input("Enter move: ").strip().lower()

            if command == "exit":
                break
            elif command == "list":
                print("Available moves:")
                for move in MovementStyle:
                    print(f" - {move.name.lower()}")
            else:
                try:
                    move_enum = MovementStyle[command.upper()]
                    robot.drive(move_enum)
                    time.sleep(4)
                    robot.stop()
                except KeyError:
                    print("Unknown command. Type 'list' to see valid movements.")
    except KeyboardInterrupt:
        print("\nManual stop.")
    finally:
        print("Shutting down...")
        robot.stop()
        io.stop()

    