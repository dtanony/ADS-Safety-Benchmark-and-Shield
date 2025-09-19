import numpy as np
import utils

class Vehicle:
    """
    Represent vehicles
    """
    def __init__(self, size, position, heading_deg, center_offset):
        """
        Parameters:
        - size: (length, width) of the rectangle
        - position: (x, y) in world frame, the reference point
        - heading_deg: in degrees, counterclockwise, 0 at Ox(+) ray. Note that:
        - center: (a, b) vector in vehicle frame: offset from reference point to center,
                  where a is wrt the vehicle length, and b is wrt the vehicle width
        """
        self.size = np.array(size)
        self.position = np.array(position)
        self.heading_deg = heading_deg
        self.center_offset = np.array(center_offset)

    def get_center(self):
        """Return the center point of the vehicle in world coordinates."""
        theta = np.deg2rad(self.heading_deg)
        rot = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        return self.position + rot @ self.center_offset

    def get_mid_front(self):
        mid_front_offset = self.center_offset + np.array((self.size[0]/2, 0))
        theta = np.deg2rad(self.heading_deg)
        rot = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        return self.position + rot @ mid_front_offset

    def get_vertices(self):
        """Return the 4 corner points of the vehicle in world coordinates."""
        width, length = self.size
        dx = width / 2
        dy = length / 2

        # Local rectangle corners (FR, FL, RL, RR) relative to vehicle center
        local_vertices = np.array([
            [ dx,  dy],  # front-right
            [-dx,  dy],  # front-left
            [-dx, -dy],  # rear-left
            [ dx, -dy],  # rear-right
        ])

        # Rotation matrix (counter-clockwise heading)
        theta = np.deg2rad(self.heading_deg)
        rot = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        # Rotate and translate to world frame
        center_world = self.get_center()
        world_vertices = (rot @ local_vertices.T).T + center_world
        return world_vertices

    def get_rotated_rect(self):
        vertices = self.get_vertices()
        return vertices[:, 0], vertices[:, 1]

    def advance(self, vel, dt):
        """Advance the vehicle by $dt time and $vel velocity."""
        self.position += vel * dt

    @classmethod
    def is_collision(self, veh1, veh2):
        return utils.is_collision(veh1.get_vertices(), veh2.get_vertices())