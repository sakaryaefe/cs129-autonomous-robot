"""
proximity_sensor.py

This module defines classes for interfacing with HC-SR04 ultrasonic sensors using pigpio
callbacks for precise distance measurements. It enables real-time proximity sensing for
robot navigation and obstacle avoidance behaviors.

Includes:
- Ultrasound class: Handles triggering and echo timing using rising/falling edge callbacks.
- ProximitySensor class: Combines left, middle, and right ultrasonic sensors for multi-directional sensing.
- Main test loop for triggering sensors and printing distance readings in meters.
"""

# proximity_sensor.py
# Threaded ultrasound triggering for real-time obstacle sensing

import pigpio
import time
import numpy as np
import constants
import threading

# === Single Ultrasonic Sensor Handler ===
class Ultrasound:
    """
    Represents a ultrasonic sensor.
    It handles sending the trigger signal and measuring the echo time
    using rising and falling edge callbacks.
    """
    def __init__(self, io, pintrig, pinecho):
        self.io = io             # pigpio interface
        self.pintrig = pintrig   # Trigger pin
        self.pinecho = pinecho   # Echo pin
        self.risetick = 0        # Timestamp of rising edge
        self.distance = np.inf   # Distance in meters

        # Configure GPIO modes
        io.set_mode(pintrig, pigpio.OUTPUT)
        io.set_mode(pinecho, pigpio.INPUT)
        
        # Register callbacks for rising and falling edge detection on echo pin
        self.cbrise = io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        self.cbfall = io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)

    def trigger(self):
        """
        Sends a 10s trigger pulse to start ultrasonic measurement.
        """
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        """
        Called when echo pin goes HIGH. Records start time.
        """
        self.risetick = ticks

    def falling(self, pin, level, ticks):
        """
        Called when echo pin goes LOW. Calculates distance based on time delta.
        """
        delt = ticks - self.risetick
        if delt < 0:
            delt += 2 ** 32  # Handle 32-bit timer wraparound
        # Convert time (in s) to distance (in meters)
        self.distance = delt * 343 / 2_000_000  # Speed of sound: 343 m/s

    def read(self):
        """
        Returns last measured distance in meters. Returns -1 if no valid reading.
        """
        return self.distance if self.distance is not None else -1

# === Multi-Sensor Proximity Handler ===
class ProximitySensor:
    def __init__(self, io):
        """
        Initializes 3-directional ultrasonic sensor array and starts trigger thread.
        """
        self.io = io
        self.left = Ultrasound(io, constants.PIN_TRIG_LEFT, constants.PIN_ECHO_LEFT)
        self.middle = Ultrasound(io, constants.PIN_TRIG_MIDDLE, constants.PIN_ECHO_MIDDLE)
        self.right = Ultrasound(io, constants.PIN_TRIG_RIGHT, constants.PIN_ECHO_RIGHT)

        self.triggering = True  # Thread control flag
        self.thread = threading.Thread(target=self.run, name="TriggerThread")
        self.thread.start()
        time.sleep(0.1)  # Let first reading stabilize

    def run(self):
        """
        Trigger loop that periodically sends pulses to all sensors every 50 ms.
        """
        while self.triggering:
            self.trigger()
            time.sleep(0.05)  # 50 ms delay between triggers

    def shutdown(self):
        """
        Gracefully stops the trigger thread.
        """
        self.triggering = False
        self.thread.join()

    def trigger(self):
        """
        Triggers all three ultrasonic sensors simultaneously.
        """
        self.left.trigger()
        self.middle.trigger()
        self.right.trigger()

    def read(self):
        """
        Returns a tuple with distances (left, middle, right) in meters.
        """
        return (self.left.read(), self.middle.read(), self.right.read())

# === Optional: Standalone Test Execution ===
if __name__ == "__main__":
    io = pigpio.pi()
    prox = ProximitySensor(io)
    try:
        while True:
            time.sleep(0.05)
            print("Distances:", prox.read())
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        prox.shutdown()
        io.stop()