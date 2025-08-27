"""
map_module.py

This module provides the STATUS enum, movement direction dictionaries,
and classes for mapping intersections and computing optimal paths using Dijkstra's algorithm.

"""

from enum import Enum
import numpy as np
import heapq
from collections import defaultdict
from constants import Event, Turn_Direction
import matplotlib
import matplotlib.pyplot as plt
import time


# === Street Status Enumeration ===
class STATUS(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4

# === Heading Direction Vectors ===
dx = {
    0: 0,  1: -1,  2: -1,  3: -1,
    4: 0,  5:  1,  6:  1,  7:  1
}

dy = {
    0: 1,  1:  1,  2:  0,  3: -1,
    4: -1, 5: -1,  6:  0,  7:  1
}

# === Color Mapping for Visualization ===
status_colors = {
    STATUS.UNKNOWN: 'black',
    STATUS.NONEXISTENT: 'lightgray',
    STATUS.UNEXPLORED: 'blue',
    STATUS.DEADEND: 'red',
    STATUS.CONNECTED: 'green'
}


# === Intersection Class ===
class Intersection:
    def __init__(self, x, y):
        self.x = x  # Grid x-coordinate
        self.y = y  # Grid y-coordinate
        self.streets = [STATUS.UNKNOWN for _ in range(8)]  # 8 possible directions
        self.blocked = [False] * 8 # 8 possible directions
        self.dijkstra_cost = float('inf')  # Cost for Dijkstra
        self.optimal_direction = None      # Best direction to goal
        self.id = None


# === Map Class for Robot Navigation ===
class Map:
    def __init__(self):
        self.intersections = {}  # Stores (x, y): Intersection

    def getintersection(self, x, y):
        """Returns the intersection at (x, y), creating it if necessary."""
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)]
    
    def block_intersection(self, x, y, h):
        inter = self.getintersection(x, y)
        inter.blocked[h] = True
        if (x + dx[h], y + dy[h]) in self.intersections:
            next_inter = self.getintersection(x + dx[h], y + dy[h])
            next_inter.blocked[(h + 4) % 8] = True
            
    def clear_blockages(self, x=None, y=None):
        """Clear all blockage flags from every intersection."""
        for inter in self.intersections.values():
            inter.blocked = [False] * 8
        print("[DEBUG] All blockages cleared.")
        
        
    def visualize_map(self, with_robot=False, xbot=0, ybot=0, hbot=0):
        """Visualizes the map and optionally the robot position."""
        plt.clf()
        plt.axes()
        plt.gca().set_xlim(-6, 6)
        plt.gca().set_ylim(-6, 6)
        plt.gca().set_aspect('equal')

        # Draw grid background
        for x in range(-10, 10):
            for y in range(-10, 10):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)

        # Draw intersections and their streets
        for (x, y), intersection in self.intersections.items():
            for h in range(8):
                status = intersection.streets[h]
                xfrom, yfrom = x, y
                xto = x + dx[h] * 0.5
                yto = y + dy[h] * 0.5
                if intersection.blocked[h]:
                    plt.plot([xfrom, xto], [yfrom, yto], color='grey', linestyle='dashed')
                else:
                    status = intersection.streets[h]
                    plt.plot([xfrom, xto], [yfrom, yto], color=status_colors[status])

        # Draw robot arrow
        if with_robot:
            angle_rad = np.pi / 2 + hbot * (np.pi / 4)
            dx_arrow = 0.2 * np.cos(angle_rad)
            dy_arrow = 0.2 * np.sin(angle_rad)
            plt.arrow(xbot, ybot, dx_arrow, dy_arrow, head_width=0.3, head_length=0.1, color='magenta')

        plt.pause(0.001)
        
     
    def invalidate_diagonals(self, x, y, heading, outcome_is_end):
        """
        Invalidate diagonal headings depending on pull-forward outcome.
        - If not END: remove 1 (45) and 3 (135)
        - If END: remove only 3 (135)
        """
        inter = self.getintersection(x, y)
        print("outcome end: ")
        print(outcome_is_end)
        if not outcome_is_end:
            diagonals = [(heading - 1) % 8, (heading + 1) % 8,
                        (heading - 3) % 8, (heading + 3) % 8]
        else:
            diagonals = [(heading - 3) % 8, (heading + 3) % 8]

        print(diagonals)
        for h in diagonals:
            if inter.streets[h] != STATUS.CONNECTED:
                inter.streets[h] = STATUS.NONEXISTENT
                print(f"[RULE] Marked heading {h} as NONEXISTENT at ({x},{y}) based on outcome={outcome_is_end}")

    def further_invalids(self, x, y, heading_old, heading_new):
        """
        Invalidates 45 diagonals between two streets if turn skipped them.
        Applies to turns of 90 (2 steps) or more.
        """
        inter = self.getintersection(x, y)
        turn_number = (heading_new - heading_old) % 8
        if turn_number == 0:
            return

        steps = turn_number if turn_number < 5 else -(8 - turn_number)

        if abs(steps) >= 2:
            diagonals = [(heading_old + 1) % 8, (heading_old - 1) % 8]
            for h in diagonals:
                inter.streets[h] = STATUS.NONEXISTENT
                
    def map_direction(self, x, y, curr_heading, direction):
        """
        Ensures when turned into a street we saw before, it doesn't mess up the angle
        """
        step = 1 if direction == Turn_Direction.LEFT else -1
        change = step
        h = (curr_heading + change) % 8
        while h != curr_heading:
            status = self.getintersection(x,y).streets[h]
            if status in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.UNEXPLORED]:
                return change
            elif status == STATUS.UNKNOWN:
                return None
            else:
                change += step
                h = (curr_heading + change) % 8
        return None
            
    def update_map_after_move(self, x_old, y_old, heading, x_new, y_new):
        """Marks the used streets as CONNECTED between intersections."""
        self.getintersection(x_old, y_old).streets[heading] = STATUS.CONNECTED
        self.getintersection(x_new, y_new).streets[(heading + 4) % 8] = STATUS.CONNECTED

    def update_map_after_turn(self, x, y, turn_number, heading):
        """Updates the map after a turn by marking skipped directions."""
        step = 1 if turn_number > 0 else -1
        for i in range(1, abs(turn_number)):
            skipped = (heading + i * step) % 8
            self.getintersection(x, y).streets[skipped] = STATUS.NONEXISTENT

        # If final direction is valid, mark it as UNEXPLORED
        final_h = (heading + turn_number) % 8
        final_status = self.getintersection(x, y).streets[final_h]
        if final_status not in [STATUS.DEADEND, STATUS.CONNECTED]:
            self.getintersection(x, y).streets[final_h] = STATUS.UNEXPLORED

    def update_map_after_uturn(self, x, y, heading):
        """Marks a direction as DEADEND after confirming it's blocked."""
        self.getintersection(x, y).streets[heading] = STATUS.DEADEND

    def update_map_after_pull_forward(self, x, y, heading, outcome):
        #print(outcome)
        """Updates the map based on result after pull-forward exploration."""
        if outcome == Event.END:
            # Only mark as NONEXISTENT if it was UNKNOWN
            if self.getintersection(x, y).streets[heading] == STATUS.UNKNOWN:
                self.getintersection(x, y).streets[heading] = STATUS.NONEXISTENT
                print(f"[PULL] Marked heading {heading} as NONEXISTENT at ({x}, {y})")
        # But don't downgrade UNEXPLORED or CONNECTED
        else:
            if self.getintersection(x, y).streets[heading] not in [STATUS.DEADEND, STATUS.CONNECTED]:
                self.getintersection(x, y).streets[heading] = STATUS.UNEXPLORED
                print(f"[PULL] Marked heading {heading} as UNEXPLORED at ({x}, {y})")
            
    def showwithrobot(self, xbot, ybot, hbot):
        """Display the map with robot arrow."""
        self.visualize_map(with_robot=True, xbot=xbot, ybot=ybot, hbot=hbot)

    def show(self):
        """Display the map without robot."""
        self.visualize_map(with_robot=False)
    
    def get_closest_inter(self, prize_id, distances):
        """Find the intersection with the smallest distance to a specified prize"""
        min_dist = np.inf
        min_inter = None
        for (x, y), inter in self.intersections.items():
            inter_id = inter.id
            if inter_id in distances and prize_id in distances[inter_id]:
                dist = distances[inter_id][prize_id]["distance"]
                if dist < min_dist:
                    min_dist = dist
                    min_inter = inter
        return min_inter
        
    def prize_found(self, prize_id, distances):
        """Check if at least 2 intersections report being ~0.5 units from the prize."""
        inter_list = []
        for (x, y), inter in self.intersections.items():
            inter_id = inter.id
            if inter_id in distances and prize_id in distances[inter_id]:
                if distances[inter_id][prize_id]["distance"] == 0.5:
                    inter_list.append((x, y))
            if len(inter_list) >= 2:
                return inter_list
        return None
        
    def show_dijkstra(self):
        plt.clf()
        plt.axes()
        plt.gca().set_xlim(-6, 6)
        plt.gca().set_ylim(-6, 6)
        plt.gca().set_aspect('equal')

        # Draw grid background
        for x in range(-10, 10):
            for y in range(-10, 10):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)

        # Draw intersections and their streets
        for (x, y), intersection in self.intersections.items():
            for h in range(8):
                status = intersection.streets[h]
                xfrom, yfrom = x, y
                xto = x + dx[h] * 0.5
                yto = y + dy[h] * 0.5
                if intersection.blocked[h]:
                    plt.plot([xfrom, xto], [yfrom, yto], color='red', linestyle='dashed')
                else:
                    status = intersection.streets[h]
                    plt.plot([xfrom, xto], [yfrom, yto], color=status_colors[status])
           
            if intersection.optimal_direction:
                angle_rad = np.pi / 2 + intersection.optimal_direction * (np.pi / 4)
                dx_arrow = 0.2 * np.cos(angle_rad)
                dy_arrow = 0.2 * np.sin(angle_rad)
                plt.arrow(x, y, dx_arrow, dy_arrow, head_width=0.3, head_length=0.1, color='magenta')
        plt.savefig(f"dmap.png")
        plt.close()
        
    
    def dijkstra(self, goal_x, goal_y):
        """Compute shortest paths from all intersections to a goal using Dijkstra."""
        self.reset_dijkstra()
        goal = self.getintersection(goal_x, goal_y)
        goal.dijkstra_cost = 0
        heap = [(0, goal_x, goal_y)]

        while heap:
            cost, x, y = heapq.heappop(heap)
            curr = self.getintersection(x, y)

            for h in range(8):
                # Must be CONNECTED on both ends
                if curr.streets[h] != STATUS.CONNECTED:
                    continue

                # Must NOT be blocked on either end
                if curr.blocked[h]:
                    continue

                nx, ny = x + dx[h], y + dy[h]
                neighbor = self.getintersection(nx, ny)

                new_cost = cost + np.sqrt(dx[h] ** 2 + dy[h] ** 2)

                if new_cost < neighbor.dijkstra_cost:
                    neighbor.dijkstra_cost = new_cost
                    neighbor.optimal_direction = (h + 4) % 8  # Opposite direction
                    heapq.heappush(heap, (new_cost, nx, ny))

        print(f"[INFO] Dijkstra complete from goal ({goal_x}, {goal_y})")
        

    def reset_dijkstra(self):
        """Resets all intersections' Dijkstra-related fields."""
        for inter in self.intersections.values():
            inter.dijkstra_cost = float('inf')
            inter.optimal_direction = None

    def set_street(self, x, y, heading, new_state):
        """Safely sets a street's state, obeying state transition rules."""
        inter = self.getintersection(x, y)
        current_state = inter.streets[heading]

        # Prevent illegal downgrades
        if current_state == STATUS.CONNECTED and new_state in [STATUS.NONEXISTENT, STATUS.UNEXPLORED]:
            print(f"[WARN] Cannot downgrade CONNECTED to {new_state.name} at ({x},{y}) heading {heading}")
            return
        if current_state == STATUS.DEADEND and new_state == STATUS.NONEXISTENT:
            print(f"[WARN] Cannot downgrade DEADEND to NONEXISTENT at ({x},{y}) heading {heading}")
            return
        if current_state == STATUS.NONEXISTENT and new_state in [STATUS.CONNECTED, STATUS.DEADEND]:
            print(f"[WARN] Cannot upgrade NONEXISTENT to {new_state.name} at ({x},{y}) heading {heading}")
            return

        inter.streets[heading] = new_state

        # Update neighbor?s opposing direction only if safe
        nx, ny = x + dx[heading], y + dy[heading]
        opp_heading = (heading + 4) % 8
        neighbor = self.getintersection(nx, ny)
        
        neighbor_state = neighbor.streets[opp_heading]

        if neighbor_state == STATUS.UNKNOWN:
            neighbor.streets[opp_heading] = new_state
        elif neighbor_state != new_state:
            print(f"[WARN] Inconsistent neighbor state at ({nx},{ny}) heading {opp_heading}: {neighbor_state.name} vs {new_state.name}")
            
    def explore(self,curr_x, curr_y):
        """Find the closest intersection that still has UNKNOWN or UNEXPLORED streets."""
        self.dijkstra(curr_x, curr_y)
        best = None
        best_cost = float('inf')

        for (x, y), inter in self.intersections.items():
            if inter.dijkstra_cost == float('inf'):
                continue
            for h in range(8):
                if inter.streets[h] in [STATUS.UNKNOWN, STATUS.UNEXPLORED]:
                    if inter.dijkstra_cost < best_cost and not inter.blocked[h]:
                        best = (x, y)
                        best_cost = inter.dijkstra_cost

        if best:
            print(f"[EXPLORE] Nearest unfinished intersection is at {best} with cost {best_cost}")
            return best
        else:
            print("[EXPLORE] Map complete.")
            return None