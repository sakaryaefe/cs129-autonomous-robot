"""
angle_sensor.py

This module defines the AngleSensor class, which interfaces with an analog magnetometer
via ADC to compute heading angles. It provides methods to read angles, track orientation
before and after turns, and calculate the number of 45-degree turns for use in robot navigation.
Includes raw magnetometer value readers for calibration and debugging.
"""

import math
import pigpio
import time
import constants

class AngleSensor:
    """
    AngleSensor class reads magnetic sensor values from ADC channels to compute
    the robot's heading angle. It helps track the angle before and after turning 
    or pulling forward and determines how much the robot has turned, in 45-degree steps.
    """
    
    def __init__(self, io):
        self.io = io
        self.initial_angle = None  # Store heading at intersection
        self.final_angle = None    # Store heading after pull forward

        for pin in constants.INPUT_PINS:
            self.io.set_mode(pin, pigpio.INPUT)
        self.io.set_mode(constants.LATCH, pigpio.OUTPUT)
        self.io.set_mode(constants.ADDRESS, pigpio.OUTPUT)

    def read_adc(self, address):
        """
        Read analog data from a specified ADC channel.
    
        This function sets the address line for channel selection and uses a latch pin to 
        signal the ADC to read from that address. It waits for a READY signal, then reads 
        the binary output across multiple input pins.
        """
        # Lower the latch to prepare for setting the address
        self.io.write(constants.LATCH, 0)
    
        # Set the desired ADC address (channel select)
        self.io.write(constants.ADDRESS, address)
    
        # Raise the latch to lock in the address
        self.io.write(constants.LATCH, 1)
    
        # Pulse the latch back to low, then high again to trigger ADC conversion
        self.io.write(constants.LATCH, 0)
        self.io.write(constants.LATCH, 1)
    
        # Wait until the ADC indicates the data is ready to read
        while not self.io.read(constants.READY_PIN[0]):
            continue
    
        # Read the binary output from all input pins (bitwise assembly)
        value = 0
        for bit, pin in enumerate(constants.INPUT_PINS):
            value |= self.io.read(pin) << bit  # Combine bits into full integer value
    
        return value

    def readangle(self):
        """
        Return the magnetic heading angle in degrees (-180 to +180),
        scaled properly based on measured min/max values.
        """

        mag0 = self.read_adc(0)
        mag1 = self.read_adc(1)

        # Scale to [-1.0, +1.0]
        scaled0 = 2.0 * (mag0 - constants.min_mag0) / (constants.max_mag0 - constants.min_mag0) - 1.0
        scaled1 = 2.0 * (mag1 - constants.min_mag1) / (constants.max_mag1 - constants.min_mag1) - 1.0

        # Compute angle from scaled values
        angle_rad = math.atan2(scaled0, scaled1)
        angle_deg = math.degrees(angle_rad)

        return angle_deg

    def set_initial_angle(self):
        """
        Save current heading as the initial angle at intersection.
        """
        self.initial_angle = self.readangle()

    def set_final_angle(self):
        """
        Save current heading as the final angle after turn and pull forward.
        """
        self.final_angle = self.readangle()

    def get_angle_difference(self):
        """
        Compute and return the difference between saved initial and final angles.
        Correct for wrap-around at -180/+180 degrees.
        """
        if self.initial_angle is None or self.final_angle is None:
            return 0.0

        diff = self.final_angle - self.initial_angle

        # Normalize to [-180, +180]
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff


# --- Test Functions ---

def test_angle(angle_sensor):
    """
    Continuously read and print the computed angle.
    """
    while True:
        angle = angle_sensor.readangle()
        print(f"Angle: {angle:.2f} degrees")
        time.sleep(0.2)

def test_magnetometer(angle_sensor):
    """
    Continuously read and print raw magnetometer ADC values.
    """
    while True:
        mag0 = angle_sensor.read_adc(0)
        mag1 = angle_sensor.read_adc(1)
        print(f"Mag0: {mag0}, Mag1: {mag1}")
        time.sleep(0.2)