import numpy as np
import utils

JERK_TIME = 0.6                 # time from press brake to when reach maximum deceleration
MAX_DECELERATION = 0.774 * 9.81
AEB_JERK_TIME = 0.1
AEB_MAX_DECELERATION = 0.85 * 9.81
BRAKING_PEDAL_DELAY = 0.75      # braking delay
RISK_EVAL_TIME = 0.4

class Vehicle:
    """
    Abstract class for vehicles.
    """
    def __init__(self, position, heading, velocity, size):
        """
        :param position: np array (x,y)
        :param heading: scalar, counter-clockwise, in radian
        :param velocity: np array (longitudinal vel, lateral vel)
        :param size: np array (length, width)
        """
        self.position = np.array(position)
        self.heading = heading
        self.velocity = np.array(velocity)
        self.size = np.array(size)

    def get_center(self):
        return self.position

    def get_vertices(self):
        """Return the 4 corner points of the vehicle in world coordinates."""
        length, width = self.size[0], self.size[1]
        dx = length / 2
        dy = width / 2

        # Local rectangle corners (FR, FL, RL, RR) relative to vehicle center
        local_vertices = np.array([
            [ dx, -dy],  # front-right
            [ dx,  dy],  # front-left
            [-dx,  dy],  # rear-left
            [-dx, -dy],  # rear-right
        ])

        # Rotation matrix (counter-clockwise heading)
        rot = np.array([
            [np.cos(self.heading), -np.sin(self.heading)],
            [np.sin(self.heading),  np.cos(self.heading)]
        ])

        # Rotate and translate to world frame
        return (rot @ local_vertices.T).T + self.get_center()

    def speed(self):
        return np.linalg.norm(self.velocity)

    def topright(self):
        vertices = self.get_vertices()
        return vertices[0]

    def topleft(self):
        vertices = self.get_vertices()
        return vertices[1]

    @classmethod
    def is_collision(self, veh1, veh2):
        return utils.is_collision(veh1.get_vertices(), veh2.get_vertices())

class Ego(Vehicle):
    """
    Represents an ego vehicle.
    """
    def __init__(self, position, heading, velocity, size=(2.0,5.0)):
        super().__init__(position, heading, velocity, size)
        self.decel = 0
        self.speed = np.linalg.norm(self.velocity)

class NPC(Vehicle):
    """
    Represents an NPC vehicle.
    """
    def __init__(self, position, heading, velocity, size=(2.0,4.5)):
        super().__init__(position, heading, velocity, size)
        self.speed = np.linalg.norm(self.velocity)

class Simulation:
    """
    Simulation abstract class.
    """
    def __init__(self, ego: Ego, npc: NPC, sim_step=0.02):
        self.ego = ego
        self.npc = npc
        self.sim_step = sim_step
        self.collision = False

        self.time = 0
        self.brake_activated = False
        # the moment when deciding to brake (brake is applied 0.75 seconds after this moment)
        self.brake_decision_time = -1
        self.AEB_activated = False
        # delta deceleration of human brake between two consecutive steps
        self.delta_brake_acc = MAX_DECELERATION / JERK_TIME * sim_step
        # delta deceleration of AEB brake between two consecutive steps
        self.delta_AEB_acc = 0

    def step(self):
        if Vehicle.is_collision(self.ego, self.npc):
            self.collision = True
            return
        self.ego_step()
        self.npc_step()
        self.time += self.sim_step

    def ego_step(self):
        """
        The evolution of the ego vehicle takes place.
        """
        if self.ego.speed <= 0:
            return

        # update x,y, and speed
        self.ego.position[0] += self.ego.speed * self.sim_step - 0.5 * self.ego.decel * self.sim_step ** 2
        self.ego.speed = max(self.ego.speed - self.ego.decel * self.sim_step, 0)

        # if reach 0.75 seconds of delay
        if not self.brake_activated and \
                self.brake_decision_time >= 0 and \
                self.time - self.brake_decision_time >= BRAKING_PEDAL_DELAY:
            self.brake_activated = True

        # update deceleration
        # if brake was applied, but AEB is not yet activated
        if self.brake_activated and not self.AEB_activated:
            if self.ego.decel < MAX_DECELERATION:
                self.ego.decel = min(self.ego.decel + self.delta_brake_acc, MAX_DECELERATION)
        elif self.AEB_activated:
            if self.delta_AEB_acc == 0:
                self.delta_AEB_acc = (AEB_MAX_DECELERATION - self.ego.decel) / AEB_JERK_TIME * self.sim_step
            if self.ego.decel < AEB_MAX_DECELERATION:
                self.ego.decel = min(self.ego.decel + self.delta_AEB_acc, AEB_MAX_DECELERATION)

    def npc_step(self):
        """
        The evolution of the NPC takes place.
        This depends on scenarios, so here is only abstract implementation.
        """
        if self.npc.speed <= 0:
            return

# There are two environment configurations: CARLA and AWSIM-Labs
# Depending on which environment is used, the parameters, e.g., lane width, median strip width, are set accordingly.
# By default, we use AWSIM-Labs configuration.
carla_env_config = {
    'lane_width': 3.5,
    'median_strip': 0.2,
    'ego_length': 4.5,
    'ego_width': 2.0,
    'npc_length': 3.7,
    'npc_width': 1.8
}
carla_town07_env_config = {
    'lane_width': 3.2,
    'median_strip': 0.0,
    'ego_length': 4.5,
    'ego_width': 2.0,
    'npc_length': 3.7,
    'npc_width': 1.8
}
awsim_env_config = {
    'lane_width': 3.3,
    'median_strip': 1.0,
    'ego_length': 4.9,
    'ego_width': 2.2,
    'npc_length': 4.0,
    'npc_width': 1.9
}