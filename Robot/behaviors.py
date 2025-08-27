"""
behaviors.py

This module defines the Behaviors class, which encapsulates high-level autonomous and 
manual behaviors for a line-following robot. It integrates sensor readings and motor 
control to execute actions such as line-following, detecting intersections and ends, 
performing turns, scanning surroundings, and correcting heading.

Includes:
- Feedback-based line following with hysteresis filtering
- Dead-end and intersection detection
- Fixed-angle and spin-based turning using magnetometer feedback
- Pull-forward after turn and re-centering routines
- Environmental scanning for unknown directions
"""
import time
import constants
from constants import Side_State, Event, Turn_Direction, MovementStyle

class Behaviors:
    """
    The Behaviors class defines high-level actions for the robot...
    """

    def __init__(self, drive, line_sensor, angle_sensor, proxy_sensor):
        """
        Initializes the Behaviors class.

        Args:
            drive: The drive system object.
            sensor: The line sensor object.
            angle_sensor: The angle sensor object.
            proxy_sensor: The proximity sensor object.
            shared: The shared data object for inter-thread communication.
        """
        self.drive = drive
        self.line_sensor = line_sensor
        self.angle_sensor = angle_sensor
        self.proxy_sensor = proxy_sensor

    def line_following(self):
        """
        Follows the line using IR sensors and basic feedback control,
        checking for shared flags.
        """
        level_intersection = 0.0
        level_end = 0.0
        level_side = 0.0
        side_state = Side_State.CENTER
        t_last = time.time()
        t_wait = None
        obstacle_timer_start = None
        prize_detected = False
        level_prize = 0.0

        while True:
            _, mid, _ = self.proxy_sensor.read()
            if mid < 0.15:
                self.drive.stop()
                continue
                
            heading_reading = self.angle_sensor.readangle()
            L, M, R = self.line_sensor.read()
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now

            if not (level_side > constants.THRESHOLD or level_side < -constants.THRESHOLD) or (L, M, R) != (0, 0, 0):
                if (L, M, R) in [(1, 1, 1), (0, 0, 0)]:
                    raw_side = 0
                elif (L, M, R) == (0, 0, 1):
                    raw_side = 1
                elif (L, M, R) == (1, 0, 0):
                    raw_side = -1
                elif (L, M, R) == (0, 1, 1):
                    raw_side = 0.5
                elif (L, M, R) == (1, 1, 0):
                    raw_side = -0.5
                elif (L, M, R) in [(0, 1, 0), (1, 0, 1)]:
                    raw_side = 0

                level_side += dt / constants.T_SIDE * (raw_side - level_side)

            if level_side > constants.THRESHOLD:
                side_state = Side_State.LEFT
            elif level_side < -constants.THRESHOLD:
                side_state = Side_State.RIGHT
            elif -constants.HYSTERESIS < level_side < constants.HYSTERESIS:
                side_state = Side_State.CENTER

            raw_i = 1.0 if (L == 1 and M == 1 and R == 1) else 0.0
            level_intersection += dt / constants.T_DETECTOR * (raw_i - level_intersection)

            raw_e = 1.0 if (L == 0 and M == 0 and R == 0) else 0.0
            level_end += dt / constants.T_END * (raw_e - level_end)

            if level_intersection > 0.55:
                self.drive.stop()
                return Event.INTERSECTION
                
            if level_end > 0.52 and side_state == Side_State.CENTER:
                self.drive.stop()
                return Event.END

            # executing the u-turn when obstacle is seen for a certain amount of time
            if mid < 0.15:
                # Start timer if it's the first detection
                if obstacle_timer_start is None:
                    obstacle_timer_start = t_now
                    print("[INFO] Obstacle detected, starting timer...")

                self.drive.stop()  # Stop moving while waiting for obstacle clear or U-turn

                # Check if 4 seconds have passed to trigger U-turn
                if t_now - obstacle_timer_start >= 4.0:
                    print("[INFO] Executing U-turn due to obstacle...")
                    self.angle_sensor.set_initial_angle()
                    target_angle = (self.angle_sensor.initial_angle + 180) % 360

                    spin_timeout = 5.0
                    spin_start_time = time.time()

                    while time.time() - spin_start_time < spin_timeout:
                        self.drive.drive(MovementStyle.LEFT_SPIN)
                        current_angle = self.angle_sensor.readangle()
                        delta = (current_angle - target_angle + 540) % 360 - 180
                        if abs(delta) < 5:
                            print("[INFO] U-turn completed.")
                            break
                        time.sleep(0.05)  # Allow sensor updates and reduce CPU usage

                    self.drive.stop()
                    obstacle_timer_start = None  # Reset timer after U-turn
                continue  # Skip rest of loop and check again
            elif mid < 0.25:
                # Minor obstacle detection, prepare for possible U-turn
                if obstacle_timer_start is None:
                    obstacle_timer_start = t_now
                    print("[INFO] Obstacle warning (mid<0.25), starting timer...")
                self.drive.stop()
                continue
            else:
                # Reset timer if obstacle clears
                if obstacle_timer_start is not None:
                    print("[INFO] Obstacle cleared, resetting timer.")
                obstacle_timer_start = None

            # prize detection logic            
            raw_p = 1.0 if (L, M, R) == (1, 0, 1) else 0.0
            level_prize += dt / constants.T_DETECTOR * (raw_p - level_prize)

            if level_prize > 0.55 and not prize_detected:
                print("Prize marker detected (1 0 1)!")
                prize_detected = True  # Prevent multiple detections

            if (L, M, R) == (1, 1, 0):
                style = MovementStyle.LEFT_STEER
            elif (L, M, R) == (1, 0, 0):
                style = MovementStyle.LEFT_TURN
            elif (L, M, R) == (0, 1, 1):
                style = MovementStyle.RIGHT_STEER
            elif (L, M, R) == (0, 0, 1):
                style = MovementStyle.RIGHT_TURN
            elif (L, M, R) == (0, 0, 0):
                if side_state == Side_State.LEFT:
                    style = MovementStyle.RIGHT_SPIN
                elif side_state == Side_State.CENTER:
                    style = MovementStyle.FORWARD
                else:
                    style = MovementStyle.LEFT_SPIN
            else:
                style = MovementStyle.FORWARD

            self.drive.drive(style)

    def pull_forward(self):
        """
         Moves the robot forward for a given duration, checking shared flags.
        """
        print("pulling forward")
        level = 0
        self.drive.drive(MovementStyle.FORWARD)
        t0 = time.time()
        t_last = t0
        while time.time() - t0 < 0.26:
            L, M, R = self.line_sensor.read()
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now
            raw = 1.0 if (L, M, R) == (0, 0, 0) else 0.0
            level += dt / 0.01 * (raw - level)

        self.drive.stop()
        print("level")
        print(level)
        if level >= 0.0:
            return Event.END
        return Event.INTERSECTION

    def magneton(self):
        self.electromagnet.on()

    def magnetoff(self):
        self.electromagnet.off()
    
    def turn_fixed_angle(self, direction, degrees=45):
        """
        Rotates the robot a fixed angle using angle sensor, checking shared flags.
        """
        self.angle_sensor.set_initial_angle()
        start_angle = self.angle_sensor.initial_angle
        target_diff = degrees if direction == Turn_Direction.RIGHT else -degrees

        if direction == Turn_Direction.LEFT:
            self.drive.drive(MovementStyle.LEFT_SPIN)
        else:
            self.drive.drive(MovementStyle.RIGHT_SPIN)

        while True:
            current_angle = self.angle_sensor.readangle()
            delta = current_angle - start_angle

            # Normalize to [-180, 180]
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360

            if (direction == Turn_Direction.RIGHT and delta >= target_diff) or \
               (direction == Turn_Direction.LEFT and delta <= target_diff):
                break

        self.drive.stop()

    def observe_line(self, timeout=1.5):
        """
        Spins in place to observe if a line is detected within a timeout,
        checking shared flags.
        """
        found_line = False
        level = 0.0
        t_last = time.time()
        t_start = t_last

        while True:
            L, M, R = self.line_sensor.read()
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now

            raw = 1.0 if 1 in (L, M, R) else 0.0
            level += dt / constants.T_TURN_2 * (raw - level)
            if level > constants.T_TURN_2:
                found_line = True
                break
            if t_now - t_start > timeout:
                break

            self.drive.drive(MovementStyle.RIGHT_SPIN) # Keep spinning while observing

        self.drive.stop()
        return found_line

    def check_blockage(self, heading):
        """
        Checks the latest ultrasound reading and returns whether the
        forward-facing street is blocked, checking shared flags.
        """
        _, mid, _ = self.proxy_sensor.read()
        if heading % 2 != 0:
            return mid < 0.6
        return mid < 0.4
        
    def get_turn_number(self, direction, t_start, t_last):
        """
        Computes number of 45 turn steps (positive = left, negative = right)
        using BEST = weighted average of MAG and TIME estimates.
        """
        angle_diff = self.angle_sensor.get_angle_difference()
        time_diff = t_last - t_start
        avg = (time_diff + angle_diff) / 2
        #print(f"DIFF: {time_diff}")
        
        # Estimate from magnetometer
        mag_turn_number = round(abs(angle_diff) / 45.0)
        if direction == Turn_Direction.RIGHT:
            mag_turn_number = -mag_turn_number
        
        # Estimate from time
        if time_diff <= 0.65:
            time_turn_number = 1
        elif time_diff <= 1:
            time_turn_number = 2
        elif time_diff <= 1.35:
            time_turn_number = 3
        elif time_diff <= 1.65:
            time_turn_number = 4
        elif time_diff <= 1.90:
            time_turn_number = 5
        elif time_diff <= 2.3:
            time_turn_number = 6
        else:
            time_turn_number = 8
            
            
        if direction == Turn_Direction.RIGHT:
            time_turn_number = -time_turn_number
        
        # Disregarding the angle differences in the cases where no turn is taking place
        if direction == Turn_Direction.STRAIGHT:
            angle_diff = 0
            time_turn_number = 0

        # Weighted BEST formula
        if mag_turn_number == 0 and time_diff > 2.3:
            mag_turn_number = 8
        elif mag_turn_number == 1 and time_diff > 2.3:
            mag_turn_number = 8
        elif mag_turn_number == -1 and time_diff > 2.3:
            mag_turn_number = -8 
        
        print(time_diff)
        print("mag: " + str(mag_turn_number))
        print("time: " + str(time_turn_number))
        best_turn_number = 0.05 * mag_turn_number + 0.95 * time_turn_number 
        turn_number = round(best_turn_number)

        # Avoid turn_number = 0 unless both inputs were 0
        if turn_number == 0 and (mag_turn_number != 0):
            turn_number = 1 if time_turn_number > 0 else -1

        #print(f"[INFO] Final turn number: {turn_number}")
        return turn_number

    def turning_spinning(self, direction):
        """
        Spins the robot in place until re-centered on the line,
        checking shared flags.
        """
        self.angle_sensor.set_initial_angle()

        level_turn = 0.0
        t_last = time.time()
        t_begin = t_last
        phase = 1

        while True:

            L, M, R = self.line_sensor.read()
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now

            if phase == 1:
                raw_t = 1.0 if 1 not in (L, M, R) else 0.0
                level_turn += dt / constants.T_TURN * (raw_t - level_turn)

                if level_turn > constants.T_TURN:
                    level_turn = 0.0
                    phase = 2
            else:
                raw_t = 1.0 if 1 in (L, M, R) else 0.0
                level_turn += dt / constants.T_TURN * (raw_t - level_turn)

                if level_turn > constants.T_TURN_2:
                    break

            spin_style = MovementStyle.LEFT_SPIN if direction == Turn_Direction.LEFT else MovementStyle.RIGHT_SPIN
            self.drive.drive(spin_style)

        self.drive.stop()

        level = 0
        while True:

            L, M, R = self.line_sensor.read()
            raw = 1.0 if (L, M, R) == (0, 1, 0) else 0.0
            level += dt / constants.T_END * (raw - level)

            if level > 0.25:
                break

            if (L, M, R) in [(1, 0, 0), (1, 1, 0)]:
                self.drive.drive(MovementStyle.LEFT_SPIN)
            elif (L, M, R) in [(0, 0, 1), (0, 1, 1)]:
                self.drive.drive(MovementStyle.RIGHT_SPIN)

        self.drive.stop()

        self.angle_sensor.set_final_angle()
        self.angle_sensor.get_angle_difference()
        turn_number = self.get_turn_number(direction, t_begin, t_last)

        return turn_number