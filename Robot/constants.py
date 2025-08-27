"""
constants.py

This module defines all hardware pin assignments, robot movement parameters,
filtering constants, thresholds, and enums used throughout the robot control system.

Includes:
- Motor and sensor GPIO pin mappings
- MovementStyle Enum with predefined motor power settings for line-following and turning
- Time constants for filtering and transition detection
- Threshold values for sensor signal interpretation
- Enums for side state, turn direction, and navigation events
- Magnetometer calibration constants for angle sensor scaling
"""
from enum import Enum

# === Movement Styles Dictionary ===
class MovementStyle(Enum):
    FORWARD = {"motor_r": 0.94,  "motor_l": 1}          # Straight movement

    # Left side corrections
    LEFT_VEER  = {"motor_r": 0.82,  "motor_l": 0.77}    # Slight left correction  
    LEFT_STEER = {"motor_r": 0.82,  "motor_l": 0.70}    # Moderate left turn   
    LEFT_TURN  = {"motor_r": 0.85, "motor_l": 0.45}     # Sharper left turn 
    LEFT_HOOK  = {"motor_r": 0.85,  "motor_l": 0.00}    # Hard left hook
    LEFT_SPIN  = {"motor_r": 0.80,  "motor_l": -0.84}   # Spin in place to the left

    # Right side corrections
    RIGHT_VEER  = {"motor_r": 0.69,  "motor_l": 0.835}  # Slight right correction    
    RIGHT_STEER = {"motor_r": 0.61,  "motor_l": 0.84}   # Moderate right turn  
    RIGHT_TURN  = {"motor_r": 0.37,  "motor_l": 0.85}   # Sharper right turn  
    RIGHT_HOOK  = {"motor_r": 0.00,  "motor_l": 0.88}   # Hard right hook
    RIGHT_SPIN  = {"motor_r": -0.80, "motor_l": 0.84}   # Spin in place to the right


# === Motor Pin Assignments ===
PIN1 = 5   # Right motor A
PIN2 = 6   # Right motor B
PIN3 = 7   # Left motor A
PIN4 = 8   # Left motor B


# === IR Sensor Pin Assignments ===
PINL = 14  # Left IR sensor
PINM = 15  # Middle IR sensor
PINR = 18  # Right IR sensor

# === Proximity Sensor Pin Assignments ===
PIN_TRIG_LEFT = 13
PIN_ECHO_LEFT = 16
PIN_TRIG_MIDDLE = 19
PIN_ECHO_MIDDLE = 20
PIN_TRIG_RIGHT = 26
PIN_ECHO_RIGHT = 21

# === Electromagnet Sensor Pin Assignment ===

PIN_MAGNET = 28

# === Filtering Constants ===
T_DETECTOR = 0.13
T_SIDE = 0.02

T_END   = 0.5     # Time constant for end-of-street detection
T_TURN  = 0.3     # Time constant for leaving the line for turning
T_TURN_2 = 0.6    # Time constant for finding the next line after turning

# === Thresholds ===
THRESHOLD  = 0.63      # Upper bound for ON
HYSTERESIS = 0.37      # Lower bound for OFF
TURN_THRES = 0.22      #Turning Threshold
END_LEVEL_THRES = 0.8  # End Threshold    ''''
OFF_DURATION = 0.35    # Offline Duration  ''''


# === Enums ===
class Side_State(Enum):
    LEFT   = 1
    CENTER = 2
    RIGHT  = 3

class Event(Enum):
    INTERSECTION   = 1
    END            = 2
    PRIZE_DETECTED = 3
    
class Turn_Direction(Enum):
    LEFT     = "left"
    RIGHT    = "right"
    STRAIGHT = "straight"
    STOP     = "stop"

# GPIO Pin assignments
LATCH = 27
ADDRESS = 4
INPUT_PINS = [9, 10, 11, 12, 22, 23, 24, 25]
READY_PIN = [17]

min_mag0 = 70
max_mag0 = 171
min_mag1 = 70
max_mag1 = 168
