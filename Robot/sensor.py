"""
sensor.py

This module provides classes to interface with IR sensors used for line detection.
It allows reading individual infrared sensor values and grouping them for line-following logic.

Includes:
- ir_sensor class: Reads digital input from a single IR sensor.
- line_sensor class: Aggregates left, middle, and right IR sensors and returns their readings as a tuple.
"""

import pigpio
import sys
import time
import constants


class ir_sensor:
    """
    Represents a single infrared sensor connected to a GPIO input pin.
    """

    def __init__(self, io, pin):  
        self.io = io
        self.pin = pin
        self.io.set_mode(pin, pigpio.INPUT)

    def read(self):
        """
        Reads the current value of the IR sensor.
        Returns 0 or 1.
        """
        return self.io.read(self.pin)


class line_sensor:
    """
    Groups three IR sensors (left, middle, right) to detect line position.
    """

    def __init__(self, io, pin_l, pin_m, pin_r):  
        self.irl = ir_sensor(io, pin_l)
        self.irm = ir_sensor(io, pin_m)
        self.irr = ir_sensor(io, pin_r)

    def read(self):
        """
        Reads all three IR sensors.
        Returns a tuple: (left, middle, right)
        """
        return (self.irl.read(), self.irm.read(), self.irr.read())