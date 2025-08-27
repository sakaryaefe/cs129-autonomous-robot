"""
proximity_behaviors.py

This module implements two proximity-based robot behaviors using ultrasonic distance sensors:
herding behavior and wall-following behavior. Both rely on real-time distance measurements 
from the left, middle, and right sensors to determine movement adjustments.

Includes:
- herding_behavior: Maintains group cohesion or avoids close contact using directional rules.
- wall_following_behavior: Maintains a consistent distance from a wall using proportional control logic.
- wall_following_continuous : Instead of using set of styles to maintain the distance between the robot and the wall, it calculates the most optimal 'pwm' values to correct the robot.
- Both herding_behavior and wall_following_behavior trigger ultrasonic sensors at regular intervals and control motors using defined MovementStyles.
"""


import time
import pigpio
from proximity_sensor import ProximitySensor
from drive import DriveSystem
import numpy as np
import constants
import traceback

# === HERDING BEHAVIOR ===
def herding_behavior(shared, drive, sensor):
    """
    Robot navigates based on proximity to obstacles on left, middle, and right sensors.
    Tries to avoid close contact and move freely when no obstacle is nearby.
    """
    try:
        last_trigger_time = time.time()
        sensor.trigger()

        while True:
            # --- Check for stop/quit signals from shared data ---
            if shared.acquire():
                should_quit = shared.quit
                should_stop = shared.stop
                shared.release()

            if should_quit or should_stop:
                drive.stop() # Ensure motors stop when exiting
                break # Exit the infinite loop

            now = time.time()

            if now - last_trigger_time > 0.050:
                sensor.trigger()
                last_trigger_time = now
		
		# Read distances from all 3 directions
                d_left, d_middle, d_right = sensor.read()
                threshold = 0.20  # 20 cm

		# Decision tree based on proximity logic
                if d_middle > threshold and d_left > threshold and d_right > threshold:
                    drive.drive(constants.MovementStyle.FORWARD)
                elif d_middle > threshold and d_left < threshold and d_right < threshold:
                    drive.drive(constants.MovementStyle.FORWARD)
                elif d_middle < 0.10 and d_left > threshold and d_right > threshold:
                    drive.drive(constants.MovementStyle.FORWARD, backwards = True)
                elif d_middle < 0.10 and d_left < threshold and d_right < threshold:
                    drive.drive(constants.MovementStyle.FORWARD, backwards = True)
                elif d_middle > 0.10 and d_middle <= threshold and d_left > threshold and d_right > threshold:
                    drive.stop()
                elif d_middle > 0.10 and d_middle <= threshold and d_left < threshold and d_right < threshold:
                    drive.stop()
                elif d_middle > threshold and d_left < threshold:
                    drive.drive(constants.MovementStyle.RIGHT_STEER)
                elif d_middle > threshold and d_right < threshold:
                    drive.drive(constants.MovementStyle.LEFT_STEER)
                elif d_middle < 0.10 and d_left < threshold:
                    drive.drive(constants.MovementStyle.RIGHT_STEER, backwards = True)
                elif d_middle < 0.10 and d_right < threshold:
                    drive.drive(constants.MovementStyle.LEFT_STEER, backwards = True)
                elif d_middle > 0.10 and d_middle <= threshold and d_left < threshold:
                    print("right spin")
                    drive.drive(constants.MovementStyle.RIGHT_SPIN)
                elif d_middle > 0.10 and d_middle <= threshold and d_right < threshold:
                    drive.drive(constants.MovementStyle.LEFT_SPIN)
                    print("left spin")

            
            time.sleep(0.01)

    except BaseException as ex:
        print("Ending herding due to exception: %s" % repr(ex))
        traceback.print_exc()
	
# === DISCRETE WALL FOLLOWING ===
def wall_following_behavior(shared, drive, sensor):
    """
    Robot navigates based on proximity to obstacles on left, middle, and right sensors.
    Tries to avoid close contact and move freely when no obstacle is nearby.
    """
    try:
        d_nom = 0.30 

        last_trigger_time = time.time()
        sensor.trigger()
        out_of_bounds = 0

        while True:
            # --- Check for stop/quit signals from shared data ---
            if shared.acquire():
                should_quit = shared.quit
                should_stop = shared.stop
                shared.release()

            if should_quit or should_stop:
                drive.stop() # Ensure motors stop when exiting
                break # Exit the infinite loop

            now = time.time()
            if now - last_trigger_time > 0.050:  
                sensor.trigger()
                last_trigger_time = now

                d_left, d_middle, _ = sensor.read()
                if d_left < 0: # Invalid reading, skip this cycle
                    continue

                error = d_left - d_nom  # Compute distance error
		
		# Stop robot if too far off target for 3 consecutive cycles
                if d_left < 0.20 or d_left > 0.40:
                    out_of_bounds += 1
                    if out_of_bounds >= 3:
                        drive.stop()
                else:
                    out_of_bounds = 0 # Reset if back in bounds

		# Adjust movement based on error magnitude
                if -0.04 <= error <= 0.04:
                    drive.drive(constants.MovementStyle.FORWARD)
                elif -0.1 <= error < -0.04:
                    drive.drive(constants.MovementStyle.RIGHT_VEER)
                elif 0.04 < error <= 0.1:
                    drive.drive(constants.MovementStyle.LEFT_VEER)
                elif error > 0.1:
                    drive.drive(constants.MovementStyle.LEFT_STEER)
                elif error < -0.1:
                    drive.drive(constants.MovementStyle.RIGHT_STEER)

                if d_middle < 0.1: # Obstacle in front
                    drive.stop()

            time.sleep(0.01)

    except BaseException as ex:
        print("Ending discrete wall following due to exception: %s" % repr(ex))
        traceback.print_exc()

# === CONTINUOUS WALL FOLLOWING ===	
def wall_following_continuous(shared, drive, sensor):
    """
    Same thing as the above function just recalculates the pwms continuously.
    """
    try:
        d_nom = 0.30  # 30 cm from wall
        base_speed = 0.9  # Base speed for both motors
        k = 2  # Steering sensitivity

        last_trigger_time = time.time()
        sensor.trigger()

        while True:
            # --- Check for stop/quit signals from shared data ---
            if shared.acquire():
                should_quit = shared.quit
                should_stop = shared.stop
                shared.release()

            if should_quit or should_stop:
                drive.stop() # Ensure motors stop when exiting
                break # Exit the infinite loop

            now = time.time()
            if now - last_trigger_time > 0.050:
                sensor.trigger()
                last_trigger_time = now

                d_left, d_middle, d_right = sensor.read()

                if d_middle < 0.10: # Obstacle in front
                    drive.stop()
                else:
                    error = d_left - d_nom

                    if np.abs(error) > 0.10: # If error is too large, stop or try to recover more aggressively
                        drive.stop()
                    else:
			 # Compute new PWM values with proportional correction
                        pwm_left = max(0.0, min(1.0, base_speed - k * error))
                        pwm_right = max(0.0, min(1.0, base_speed + k * error))

                        drive.pwm(pwm_left, pwm_right)

            time.sleep(0.01)

    except BaseException as ex:
        print("Ending continuous wall following due to exception: %s" % repr(ex))
        traceback.print_exc()

if __name__ == "__main__":
    # Uncomment one of these for standalone testing
    # herding_behavior()
    # wall_following_behavior()
    wall_following_continuous()