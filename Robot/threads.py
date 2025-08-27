"""
threads.py

This module manages multi-threaded control of a robot navigation system.
It defines a shared data structure (`SharedData`) for communication between
the UI thread and the robot's main control thread. The UI thread (`run_ui`)
handles user input for controlling robot modes such as exploration, goal-seeking,
step-by-step execution, pose setting, and map saving/loading.

Main Components:
- SharedData: Thread-safe shared state object used by UI and robot logic.
- run_ui: Thread function to handle user input and update shared data accordingly.
"""

import threading
import pigpio
import time
from constants import Turn_Direction, Event
from proximity_sensor import ProximitySensor
from drive import DriveSystem
from mapping import Map, dx, dy, STATUS
from behaviors import Behaviors
from Main_Thread import Main_Thread
import constants
from angle_sensor import AngleSensor
from sensor import line_sensor

# === Shared Data Between Threads ===
class SharedData:
    """
    A thread-safe shared memory structure for communication between threads.
    Stores control flags, pose, mode commands, and goal coordinates.
    """

    def __init__(self):
        self.lock = threading.Lock()      # Used to safely access shared variables
        self.shutdown = False             # Signals the robot to shut down
        self.pause = False                # Pauses robot execution
        self.step = False                 # Enables step-by-step mode
        self.exploring = False            # True if the robot should explore automatically
        self.goal_coords = None           # Target goal (x, y) if in goal mode
        self.pose = (0, 0, 0)             # Robot's (x, y, heading)
        self.awaiting_pose = True         # Flag to wait for manual pose input
        self.command = None               # User-specified command (e.g., "left", "goal", etc.)
        self.map_file = None              # Optional map filename to load/save
        self.load = False
        self.save = False
        self.start = True
        self.clear_blockages = False
        self.robotx = 0                   # Robot's x-coordinate for ROS
        self.roboty = 0                   # Robot's y-coordinate for ROS
        self.robotheading = 0             # Robot's heading 
        self.prize_to_fetch = None
        self.game = False
        self.inter_prize_distance_dict = {}
        self.prize_info_dict = {}

    def acquire(self):
        """Acquire the internal thread lock manually."""
        return self.lock.acquire()

    def release(self):
        """Release the internal thread lock manually."""
        self.lock.release()
        
# === UI Thread Function ===
def run_ui(shared, global_map):
    """
    This function runs in a separate thread and handles user input from the console.
    It updates shared state variables with commands, goals, and control flags.
    """
    while True:
        with shared.lock:
            if shared.awaiting_pose:
                try:
                    x = int(input("Enter robot initial x: "))
                    y = int(input("Enter robot initial y: "))
                    h = int(input("Enter robot initial heading (0-7): "))
                    shared.pose = (x, y, h)
                    robotx, roboty, robotheading = (x,y,h)
                    shared.awaiting_pose = False
                except ValueError:
                    print("[ERROR] Invalid pose input. Try again.")
                continue

        cmd = input("Command? ").strip().lower()

        if cmd == "quit":
            with shared.lock:
                shared.shutdown = True
            break
        elif cmd == "explore":
            with shared.lock:
                if not shared.pause:
                    shared.exploring = True
                    shared.goal_coords = None
                else:
                    print("Robot is paused. Use 'resume' first.")
        elif cmd == "goal":
            shared.exploring = False
            try:
                x = int(input("Goal x: "))
                y = int(input("Goal y: "))
                with shared.lock:
                    if not shared.pause:
                        shared.goal_coords = (x, y)
                        shared.command = ("goal", x, y)
                        shared.awaiting_pose = False
                    else:
                        print("Robot is paused. Use 'resume' first.")
            except ValueError:
                print("Invalid goal coordinates.")
        elif cmd == "pause":
            with shared.lock:
                shared.pause = True
        elif cmd == "fetch":
            try:
                prize_id = int(input("Prize ID to fetch: "))
                with shared.lock:
                    shared.prize_to_fetch = prize_id
            except ValueError:
                print("Invalid prize ID.")
        elif cmd == "resume":
            with shared.lock:
                print("Paused state lifted.")
                shared.pause = False
        elif cmd == "clear":
            with shared.lock:
                print("Paused state lifted.")
                shared.clear_blockages = True
        elif cmd == "step":
            with shared.lock:
                if not shared.pause:
                    shared.step = True
                    shared.needs_reactivation = False
                else:
                    print("Robot is paused. Use 'resume' first.")
        elif cmd in ["left", "right", "straight"]:
            with shared.lock:
                if not shared.pause:
                    shared.command = cmd
                    shared.exploring = False
                    shared.goal_coords = None 
                else:
                    print("Robot is paused. Use 'resume' first.")
        elif cmd == "save":
            with shared.lock:
                shared.map_file = input("Enter filename to save map (e.g., map1.pickle): ").strip()
                shared.save = True
        elif cmd == "load":
            with shared.lock:
                shared.map_file = input("Filename to load: ").strip()
                shared.load = True
        elif cmd == "pose":
            shared.awaiting_pose = True
        elif cmd == "show":
            with shared.lock:
                x, y, h = shared.pose
                print(f"Current pose: x={x}, y={y}, heading={h}")
        else:
            print("Unknown command.")

if __name__ == "__main__":
    main()
    