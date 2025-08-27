"""
Main_Thread.py

This script contains the main control loop for the autonomous robot navigation system.
It coordinates manual and autonomous modes, processes user commands, manages robot
pose updates, and interfaces with the mapping system. The robot can follow lines,
navigate intersections, execute turns based on Dijkstra pathfinding, and perform
environmental exploration by prioritizing UNKNOWN and UNEXPLORED streets.

Includes:
- Manual mode for step-by-step command input (L/R/S/G/A/SM/LM/B)
- Goal-oriented mode using Dijkstras algorithm
- Autonomous mode for full map exploration
- Robot pose tracking and heading adjustment after each move
"""

import threading
from constants import Turn_Direction, Event
from mapping import dx, dy, STATUS
import time
import pickle
import numpy as np
# === Main Brain Function (runs in robot thread) ===
def Main_Thread(shared, drive, proximity, global_map, behaviors, nfc):
    """
    The primary robot control loop running in the main thread.
    Responds to shared state set by the UI thread, handling manual moves, goal navigation,
    and autonomous exploration logic.
    """
    try:
        direction = None
        while shared.start:
            result = behaviors.line_following()
            while result == Event.END:
                behaviors.turning_spinning(Turn_Direction.LEFT)
                result = behaviors.line_following()
            if result == Event.INTERSECTION:
                shared.start = False
                outcome = behaviors.pull_forward()
        while shared.awaiting_pose:
            time.sleep(0.1)
        with shared.lock:
            global_map.update_map_after_pull_forward(shared.pose[0], shared.pose[1], shared.pose[2], outcome)

        while True:
            x, y, heading = shared.pose
            global_map.showwithrobot(x, y, heading)
            with shared.lock:
                if shared.shutdown:
                    break
                pose = shared.pose
                pause = shared.pause
                step = shared.step
                direction = get_direction_from_command(shared.command)
                goal = shared.goal_coords
                exploring = shared.exploring
                prize_id = shared.prize_to_fetch
                save = shared.save
                load = shared.load
                game = shared.game
                clear = shared.clear_blockages
                distances = shared.inter_prize_distance_dict
                info = shared.prize_info_dict
                shared.command = None
                if step:
                    shared.step = False
                if clear:
                    shared.clear_blockages = False
                    global_map.clear_blockages()
                shared.robotx, shared.roboty, shared.robotheading = pose
               
            if pause:
                time.sleep(0.1)
                continue
                
            inter = global_map.getintersection(x, y)
            # === Decide Action ===
            if direction is not None:
                # Manual control
                pass
            elif goal or exploring or prize_id:
                if prize_id is not None:
                    direction = decide_fetch_direction(shared, global_map, behaviors, nfc, proximity, pose[0], pose[1], pose[2], prize_id, distances)
                elif goal:
                    direction = decide_goal_direction(shared,x, y, heading, goal, global_map)
                elif exploring:
                    direction = decide_explore_direction(shared,x, y, heading, global_map)
                    
            # === Act ===
            if direction is not None:
                if direction == Turn_Direction.STRAIGHT:
                    print(heading)
                    x, y, heading, _ = navigate_to_next_intersection(shared, behaviors, global_map, x, y, heading, nfc)
                    if result is None:
                        direction = None
                        continue
                else:
                    heading = perform_turn_and_update(shared, direction, x, y, heading, global_map, behaviors)
                direction = None 
                
            if shared.awaiting_pose:
                x = int(input("x: "))
                y = int(input("y: "))
                h = int(input("heading (0-7): "))
                with shared.lock:
                    shared.pose = (x, y, h)
                    shared.robotx = x
                    shared.roboty = y
                    shared.robotheading = h
                    shared.awaiting_pose = False
            if load:
                try:
                    with shared.lock:
                        filename = shared.map_file
                    with open(filename, 'rb') as file:
                        global_map = pickle.load(file)
                        global_map.show()
                except ValueError:
                    print("Invalid input.")
                with shared.lock:
                    shared.load = False

            if save:
                with shared.lock:
                    filename = shared.map_file
                    shared.save = False
                with open(filename, 'wb') as f:
                    pickle.dump(global_map, f)

    except BaseException as ex:
        print("[ERROR] Main loop exception:", repr(ex))
 
def update_shared_pose(shared, x, y, heading):
    """
    Updates the pose after every move.
    """
    with shared.lock:
        shared.pose = (x, y, heading)
        shared.robotx = x
        shared.roboty = y
        shared.robotheading = heading

def get_turn_direction(current_heading, target_heading):
    """
    Determines the shortest turn (left/right) to reach the target heading from the current heading.
    """
    delta = (target_heading - current_heading) % 8
    direction = Turn_Direction.LEFT if delta < 4 else Turn_Direction.RIGHT
    return delta if direction == Turn_Direction.RIGHT else -((8 - delta) % 8), direction

def perform_turn_and_update(shared, direction, x, y, heading, global_map, behaviors):
    """
    Performs a turn in the specified direction and updates the heading in the map.
    """
    steps = behaviors.turning_spinning(direction)
    result = global_map.map_direction(x, y, heading, direction)
    if result:
        if result != steps:
            print("[WARNING] Map and magnetometer do not match")
        global_map.update_map_after_turn(x, y, result, heading)
        heading = (heading + result) % 8
    else:
        global_map.update_map_after_turn(x, y, steps, heading)
        heading = (heading + steps) % 8
    update_shared_pose(shared, x, y, heading)
    return heading

def navigate_to_next_intersection(shared, behaviors, global_map, x, y, heading, nfc):
    """
    Follows the line to the next intersection. Handles blocked paths and U-turns.
    """
    if behaviors.check_blockage(heading):
        global_map.block_intersection(x, y, heading)
        print("Blocked street")
        return x, y, heading, None
    else:
        result = behaviors.line_following()
        if result == Event.INTERSECTION:
            outcome = behaviors.pull_forward()
            print(outcome)
            dx_val, dy_val = dx[heading], dy[heading]
            x_new, y_new = x + dx_val, y + dy_val
            global_map.update_map_after_move(x, y, heading, x_new, y_new)
            
            outcome_is_end = (outcome == Event.END)
            global_map.invalidate_diagonals(x_new, y_new, heading, outcome_is_end)
            
            global_map.update_map_after_pull_forward(x_new, y_new, heading, outcome)
            global_map.getintersection(x_new, y_new).id = nfc.last_read
            update_shared_pose(shared, x_new, y_new, heading)
            return x_new, y_new, heading, result
        elif result == Event.END:
            behaviors.turning_spinning(Turn_Direction.LEFT)
            global_map.update_map_after_uturn(x, y, heading)
            heading = (heading + 4) % 8
            result = behaviors.line_following()
            outcome = behaviors.pull_forward()
            print(outcome)
            
            outcome_is_end = (outcome == Event.END)
            global_map.invalidate_diagonals(x, y, heading, outcome_is_end)
            global_map.update_map_after_pull_forward(x, y, heading, outcome_is_end)
            update_shared_pose(shared, x, y, heading)
            return x, y, heading, result
            
def get_direction_from_command(cmd):
    """
    Handle manual command strings.
    """
    if cmd == "left": return Turn_Direction.LEFT
    elif cmd == "right": return Turn_Direction.RIGHT
    elif cmd == "straight": return Turn_Direction.STRAIGHT
    else: return None

def decide_goal_direction(shared, x, y, heading, goal, global_map):
    """
    Determines best direction toward a known goal using Dijkstra path.
    """
    gx, gy = goal
    if (x, y) == (gx, gy):
        print("[INFO] Reached goal.")
        with shared.lock:
            shared.goal_coords = None
        return None

    global_map.dijkstra(gx, gy)
    inter = global_map.getintersection(x, y)

    if inter.optimal_direction is None:
        print("[FAIL] No path to goal.")
        direction = decide_directed_goal_direction(shared, x, y, heading, goal, global_map)
        return direction
    if heading == inter.optimal_direction:
        return Turn_Direction.STRAIGHT
    _, direction = get_turn_direction(heading, inter.optimal_direction)
    return direction
    
def decide_fetch_direction(shared, global_map, behaviors, nfc, proximity, curr_x, curr_y, curr_heading, prize_id, distances):
    inter_list = global_map.prize_found(prize_id, distances)
    if inter_list != None:
        if (curr_x, curr_y) not in inter_list:
            print(inter_list)
            direct = global_map.get_closest_inter(prize_id, distances)
            direction = decide_goal_direction(shared, curr_x, curr_y, curr_heading, (direct.x, direct.y), global_map)
            return direction
        else:
            print("Found the intersections that the prize is in between")
            with shared.lock:
                shared.prize_to_fetch = None
                return None
    inter = global_map.getintersection(curr_x, curr_y)
    direct = global_map.get_closest_inter(prize_id, distances)
    if direct is None or curr_x == direct.x and curr_y == direct.y:
        print("[FAIL] No intersection has distance info for this prize.")
        direction = decide_explore_direction(shared, curr_x, curr_y, curr_heading, global_map)
    else:
        direction = decide_goal_direction(shared, curr_x, curr_y, curr_heading, (direct.x, direct.y), global_map)
    return direction

def decide_prize_direction(curr_heading, prize_id, info_dict):
    direction = None
    heading_of_prize = distances[prize_id]["heading"]
    direction = get_turn_direction(curr_heading, heading_of_prize)
    return direction
    
def decide_directed_goal_direction(shared, x, y, heading, goal, global_map):
    """
    Estimates best direction toward an unknown goal using vector alignment.
    """
    print("directed")
    gx, gy = goal
    inter = global_map.getintersection(x, y)

    # Goal direction vector (normalized)
    vec_to_goal = np.array([gx - x, gy - y], dtype=float)
    norm = np.linalg.norm(vec_to_goal)
    if norm == 0:
        return None  # Already at goal
    vec_to_goal /= norm

    best_score = -float('inf')
    best_heading = None

    for h in range(8):
        if inter.streets[h] in [STATUS.UNEXPLORED, STATUS.UNKNOWN] and not inter.blocked[h]:
            heading_vec = np.array([dx[h], dy[h]], dtype=float)
            heading_norm = np.linalg.norm(heading_vec)
            if heading_norm == 0:
                continue
            heading_vec /= heading_norm
            score = np.dot(vec_to_goal, heading_vec)
            print("score:")
            print(score)
            if score > best_score:
                best_score = score
                best_heading = h

    print("best heading:")
    print(best_heading)
    print("heading:")
    print(heading)
    if best_heading is None:
        print("[INFO] No valid direction for directed goal.")
        result = global_map.explore(x,y)
        if result is None:
            return None
        sgx,sgy = result
        global_map.dijkstra(sgx, sgy)
        inter = global_map.getintersection(x, y)
        print(inter.optimal_direction)
        #return inter.optimal_direction
        best_heading = inter.optimal_direction

    if best_heading == heading:
        return Turn_Direction.STRAIGHT
    _, turn_dir = get_turn_direction(heading, best_heading)
    return turn_dir

        
def decide_explore_direction(shared, x, y, heading, global_map):
    """
    Chooses the next best direction for autonomous exploration based on unfinished paths.
    """
    inter = global_map.getintersection(x, y)

    # Try straight if UNEXPLORED and not blocked
    if inter.streets[heading] == STATUS.UNEXPLORED and not inter.blocked[heading]:
        return Turn_Direction.STRAIGHT

    # Look left/right preference order
    for change in [1, -1, 2, -2, 3, -3, 4]:
        new_heading = (heading + change) % 8
        if inter.streets[new_heading] in [STATUS.UNEXPLORED, STATUS.UNKNOWN] and not inter.blocked[new_heading]:
            return Turn_Direction.LEFT if change > 0 else Turn_Direction.RIGHT

    # Use Dijkstra to nearest unexplored node
    gxgy = global_map.explore(x, y)
    if gxgy is None:
        print("[INFO] Exploration complete.")
        with shared.lock:
            shared.exploring = False
        return None

    gx, gy = gxgy
    global_map.dijkstra(gx, gy)
    inter = global_map.getintersection(x, y)
    print("inside the explore loop")
    print(inter.optimal_direction)
    if inter.optimal_direction is None:
        return None

    if inter.optimal_direction == heading:
        return Turn_Direction.STRAIGHT
    _, direction = get_turn_direction(heading, inter.optimal_direction)
    return direction