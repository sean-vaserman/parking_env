import math
import heapq
import numpy as np

class Node:
    def __init__(self, x, y, yaw, cost, path_x, path_y, path_yaw, path_gear):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = cost
        self.path_x = path_x
        self.path_y = path_y
        self.path_yaw = path_yaw
        self.path_gear = path_gear

    def __lt__(self, other):
        return self.cost < other.cost

def check_collision(x, y, yaw, obstacles, vehicle):
    circles = [
        (x, y), 
        (x + (vehicle.L / 2) * math.cos(yaw), y + (vehicle.L / 2) * math.sin(yaw)), 
        (x + vehicle.L * math.cos(yaw), y + vehicle.L * math.sin(yaw)) 
    ]
    radius = (vehicle.W / 2.0) + 0.3
    
    for (ox, oy, w, h) in obstacles:
        for (cx, cy) in circles:
            test_x = max(ox, min(cx, ox + w))
            test_y = max(oy, min(cy, oy + h))
            
            dist_x = cx - test_x
            dist_y = cy - test_y
            if math.hypot(dist_x, dist_y) <= radius:
                return True
    return False

def plan_path(start_pos, goal_pos, obstacles, vehicle):
    print("Planning fast path... please wait.")
    open_set = []
    
    def calc_heuristic(x, y):
        return math.hypot(goal_pos[0] - x, goal_pos[1] - y)

    start_node = Node(start_pos[0], start_pos[1], start_pos[2], 
                      calc_heuristic(start_pos[0], start_pos[1]), 
                      [start_pos[0]], [start_pos[1]], [start_pos[2]], [1])
    
    heapq.heappush(open_set, start_node)
    
    # Restored to 1.0 for speed
    XY_RESO = 1.0 
    YAW_RESO = np.radians(15.0)
    visited = set()

    move_step = 1.0 
    steer_angles = [-vehicle.MAX_STEER, 0, vehicle.MAX_STEER]
    directions = [1, -1] 

    while open_set:
        current = heapq.heappop(open_set)

        dist_to_goal = math.hypot(current.x - goal_pos[0], current.y - goal_pos[1])
        yaw_diff = abs((current.yaw - goal_pos[2] + math.pi) % (2 * math.pi) - math.pi)
        
        if dist_to_goal < 1.0 and yaw_diff < np.radians(15.0):
            print("Path found!")
            return current.path_x, current.path_y, current.path_yaw, current.path_gear

        state_key = (round(current.x / XY_RESO), round(current.y / XY_RESO), round(current.yaw / YAW_RESO))
        if state_key in visited:
            continue
        visited.add(state_key)

        for d in directions:
            for steer in steer_angles:
                nx = current.x + d * move_step * math.cos(current.yaw)
                ny = current.y + d * move_step * math.sin(current.yaw)
                nyaw = current.yaw + d * (move_step / vehicle.L) * math.tan(steer)
                nyaw = (nyaw + math.pi) % (2 * math.pi) - math.pi

                if not check_collision(nx, ny, nyaw, obstacles, vehicle):
                    step_cost = move_step + (2.0 if d == -1 else 0) + (0.5 if steer != 0 else 0)
                    gear_switch_cost = 2.0 if d != current.path_gear[-1] else 0
                    
                    ncost = current.cost - calc_heuristic(current.x, current.y) + step_cost + gear_switch_cost + calc_heuristic(nx, ny)
                    
                    n_path_x = current.path_x + [nx]
                    n_path_y = current.path_y + [ny]
                    n_path_yaw = current.path_yaw + [nyaw]
                    n_path_gear = current.path_gear + [d]
                    
                    heapq.heappush(open_set, Node(nx, ny, nyaw, ncost, n_path_x, n_path_y, n_path_yaw, n_path_gear))

    print("Failed to find path.")
    return [], [], [], []