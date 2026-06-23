import math
import numpy as np

class VehicleModel:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # State
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        
        # Vehicle Parameters
        self.L = 2.5  # Wheelbase (meters)
        self.W = 1.5  # Width (meters)
        self.MAX_STEER = np.radians(35.0) # Maximum steering angle
        
    def update(self, throttle, steer, dt=0.1):
        # Clip steering angle
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)
        
        # Kinematic Bicycle Model equations
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += (self.v / self.L) * math.tan(steer) * dt
        
        # Simple velocity integration (ignoring mass/inertia)
        self.v += throttle * dt
        
        # Normalize yaw to [-pi, pi]
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi