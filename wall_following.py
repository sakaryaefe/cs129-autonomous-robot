"""
wall_following_main.py

This script sets up and runs a robot wall-following system using ultrasonic
sensors. It supports both discrete and continuous control modes and runs
with multithreaded user interaction. The user can switch between modes or
shut down the robot through terminal input.

Includes:
- Wall_Shared: Shared state with thread-safe access between UI and robot logic
- run_ui: UI thread for command input (discrete, continuous, stop, quit)
- run_robot: Executes behavior based on shared mode
- main: Initializes hardware and starts both threads
"""

import pigpio
import time
import threading
from proximity_sensor import ProximitySensor
from drive import DriveSystem
import constants
from proximity_behavior import wall_following_behavior, wall_following_continuous

class Wall_Shared:
    """
    Thread-safe shared state object between UI and robot control logic.
    Contains flags for quitting, stopping, and the current wall-following mode.
    """
    def __init__(self):
        self.lock = threading.Lock()  # Lock to ensure mutual exclusion
        self.quit = False             # Flag to exit program
        self.stop = False             # Flag to pause robot movement
        self.mode = None              # Mode: "discrete" or "continuous"

    def acquire(self):
        """Acquire the internal lock manually."""
        return self.lock.acquire()

    def release(self):
        """Release the internal lock manually."""
        self.lock.release()

def run_ui(shared):
    """
    User interface thread that listens for commands:
    - 'discrete': switch to discrete wall-following mode
    - 'continuous': switch to continuous PWM-based mode
    - 'stop': pause movement
    - 'quit': shut down system
    """
    while True:
        cmd = input("Command? ").strip().lower()
        if shared.acquire():
            if cmd == 'stop':
                shared.mode = None
                shared.stop = True
            elif cmd in ['discrete', 'continuous']:
                shared.stop = False
                shared.mode = cmd
            elif cmd == 'quit':
                shared.quit = True
                shared.release()
                break
            else:
                print("Illegal command " + cmd)
            shared.release()

def run_robot(shared, drive, proximity):
    """
    Main robot control loop. Monitors shared state and runs the
    corresponding wall-following behavior (discrete or continuous).
    """
    try:
        running = True
        while running:
            if shared.acquire():
                should_quit = shared.quit
                should_stop = shared.stop
                mode = shared.mode
                shared.release()

                if should_quit:
                    running = False
                    continue

                if should_stop:
                    drive.stop()
                    time.sleep(0.1)
                    continue

                if mode == "discrete":
                    wall_following_behavior(shared, drive, proximity)
                elif mode == "continuous":
                    wall_following_continuous(shared, drive, proximity)

            time.sleep(0.05)
    except BaseException as ex:
        print("[ERROR] Exception in robot loop:", repr(ex))
    finally:
        drive.stop()

def main():
    io = pigpio.pi()
    if not io.connected:
        raise RuntimeError("[ERROR] Cannot connect to pigpio daemon")

    proximity = ProximitySensor(io)
    drive = DriveSystem(io, constants.PIN1, constants.PIN2, constants.PIN3, constants.PIN4)
    shared = Wall_Shared()

    uithread = threading.Thread(name="UIThread", target=run_ui, args=(shared,), daemon=True)
    uithread.start()

    print("[INFO] UI thread started. Type a command (discrete / continuous / stop / quit):")
    run_robot(shared, drive, proximity)

    print("[INFO] Shutting down...")
    drive.stop()
    proximity.shutdown()
    io.stop()

if __name__ == "__main__":
    main()