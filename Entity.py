import pybullet as p
import numpy as np
# Base Class for all entities in the simulation
class Entity:
    def __init__(self, mass, friction, com):
        self.mass = mass
        self.friction = friction
        self.com = com  # Center of Mass

    def apply_friction(self, plane_id):
        p.changeDynamics(plane_id, -1, lateralFriction=self.friction)

    def get_com(self):
        return self.com

# Derived class for the scooter
class OlooScooter(Entity):
    def __init__(self, mass, friction, com, wheelbase):
        super().__init__(mass, friction, com)
        self.wheelbase = wheelbase

# Derived class for the rider
class Rider(Entity):
    def __init__(self, mass, friction, com, tilt_angle):
        super().__init__(mass, friction, com)
        self.tilt_angle = tilt_angle

    def set_tilt_angle(self, rider_id, slope):
        angle_with_slope = self.tilt_angle - slope
        # Debug: Print tilt angle values
        print(f"Setting tilt angle: {self.tilt_angle}, slope: {slope}, angle_with_slope: {angle_with_slope}")
        p.resetBasePositionAndOrientation(rider_id, [0, 0, 1], p.getQuaternionFromEuler([0, 0, np.radians(angle_with_slope)]))
