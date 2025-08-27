"""
main.py

This is the main entry point for the robot control program. It initializes all
hardware interfaces (GPIO, motors, sensors), constructs mid-level behavior
controllers, and launches the high-level brain function that manages manual
and autonomous robot navigation.

Includes:
- Initialization of pigpio connection and hardware components
- Creation of DriveSystem, IR line sensors, and magnetometer angle sensor
- Setup of Behaviors and mapping system
- Launch of the UI thread to handle user commands
- Launch of the main robot control thread (Main_Thread)
- Exception-safe cleanup of motors, sensors, and GPIO on shutdown
"""

import pigpio
import sys
import threading
import constants
from sensor import line_sensor
from drive import DriveSystem
from behaviors import Behaviors
from angle_sensor import AngleSensor
from mapping import Map
from proximity_sensor import ProximitySensor
from threads import SharedData, run_ui
from Main_Thread import Main_Thread
from ros import runros
from NFC_Sensor import NFC_Sensor

import ctypes

def main():
    """
    Initializes hardware, sensors, shared data, and robot behaviors.
    Starts UI thread and runs the robot brain in the main thread.
    Cleans up resources on shutdown.
    """
     
    # Connect to pigpio daemon
    io = pigpio.pi()
    if not io.connected:
        raise RuntimeError("[ERROR] Cannot connect to pigpio daemon")		

	# Initialize map, motors, sensors, and shared state
    global_map = Map()
    proximity = ProximitySensor(io)
    drive = DriveSystem(io, constants.PIN1, constants.PIN2, constants.PIN3, constants.PIN4)
    shared = SharedData()
    line = line_sensor(io, constants.PINL, constants.PINM, constants.PINR)
    angle = AngleSensor(io)
    behaviors_instance = Behaviors(drive, line, angle, proximity)
    nfc = NFC_Sensor()

    
	# Start UI thread to listen for user commands
    uithread = threading.Thread(target=run_ui, args=(shared, global_map), daemon=True)
    uithread.start()
    
    # Start the ROS worker thread.
    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()
    
	# Run main robot control loop (manual, goal-seeking, exploration)
    Main_Thread(shared, drive, proximity, global_map, behaviors_instance, nfc)

    # End the ROS thread (send the KeyboardInterrupt exception).
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
    ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
    rosthread.join()
    
	# Clean up motors, sensors, GPIO after shutdown
    print("[INFO] Shutdown complete.")
    nfc.shutdown()
    drive.stop()
    proximity.shutdown()
    io.stop()

if __name__ == "__main__":
    main()