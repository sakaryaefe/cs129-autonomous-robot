"""
test_magnetometer.py

Tests for finding out the center points of the magnetometer, this belowwritten code was utilized to find those values.

Includes:
- test_magnetometer_centers: Continuously prints raw ADC values to help determine min/max scaling values.
- test_manual_magnetometer_spin: Prints computed heading angles while manually rotating the robot to verify angle tracking.
- Intended for development-time calibration and debugging only.
"""

import pigpio
import time
from angle_sensor import AngleSensor

def test_magnetometer_centers():
    """
    Continuously prints raw ADC values for the magnetometer X (mag0) and Y (mag1) axes.
    Used to manually observe the min/max range and determine the center values for calibration.
    """
    io = pigpio.pi()
    sensor = AngleSensor(io)

    print("Measuring magnetometer centers... (Press CTRL+C to stop)")

    while True:
        mag0 = sensor.read_adc(0)
        mag1 = sensor.read_adc(1)
        print(f"Mag0 (X): {mag0}, Mag1 (Y): {mag1}")
        time.sleep(0.2)  # slow down printout


def test_manual_magnetometer_spin():
    """
    Prints computed heading angle (in degrees) as the robot is rotated.
    Used to verify that the heading updates smoothly and accurately.
    """
    io = pigpio.pi()
    if not io.connected:
        print("could not connect")
        return
	
    angle_sensor = AngleSensor(io)
    print("rotating and press ctrl + z to stop.")
	 
    try:
        while True:
            angle = angle_sensor.readangle()
            print(f"degree angle: {angle:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("interrupted by the user")
    finally:
        io.stop()

if __name__== "__main__":
	test_magnetometer_centers()
	#test_manual_magnetometer_spin()
