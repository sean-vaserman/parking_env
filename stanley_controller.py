import math
import numpy as np

class StanleyController:
    def __init__(self, k=0.8):
        self.k = k

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def control(self, vehicle, path_x, path_y, path_yaw, path_gear, target_idx):
        """Compute steering angle for the given target waypoint.
        Waypoint advancement is handled externally (in main.py)."""
        gear = path_gear[target_idx]
        is_reversing = vehicle.v < -0.05 or (abs(vehicle.v) <= 0.05 and gear == -1)

        # Front axle lookahead for forward; rear axle for reverse
        if not is_reversing:
            track_x = vehicle.x + vehicle.L * math.cos(vehicle.yaw)
            track_y = vehicle.y + vehicle.L * math.sin(vehicle.yaw)
        else:
            track_x, track_y = vehicle.x, vehicle.y

        map_x = track_x - path_x[target_idx]
        map_y = track_y - path_y[target_idx]
        path_angle = path_yaw[target_idx]

        cte = map_x * math.sin(path_angle) - map_y * math.cos(path_angle)
        v_safe = max(abs(vehicle.v), 0.1)
        heading_error = self.normalize_angle(path_angle - vehicle.yaw)

        # Reverse: negate heading contribution (steer effect inverts with v<0),
        # CTE sign stays positive in both cases (verified by simulation).
        if not is_reversing:
            steer = heading_error + math.atan2(self.k * cte, v_safe)
        else:
            steer = -heading_error + math.atan2(self.k * cte, v_safe)

        return np.clip(steer, -vehicle.MAX_STEER, vehicle.MAX_STEER)
