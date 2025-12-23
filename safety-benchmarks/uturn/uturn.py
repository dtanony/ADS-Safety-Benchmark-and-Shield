import argparse

from common import *
from matplotlib import pyplot as plt

# average of max angles of outer and inner wheels during U-Turn
TURNING_WHEEL_ANGLE = np.pi / 6
# assume a compact car
WHEEL_BASE = 2.5

env_config = awsim_env_config

class UTurnEgo(Ego):
    def __init__(self, position, velocity, size=(2.0,5.0)):
        super().__init__(position, 0, velocity, size)

class UTurnNPC(NPC):
    def __init__(self, position, velocity, size=(2.0,4.5), wheelbase=2.5, turning_wheel_angle=np.pi/6):
        """
        :param position: center point of the car shape rectangle.
        :param velocity:
        :param size:
        :param wheelbase:
        :param turning_wheel_angle:

        Note that we maintain self.position as the center point between two front wheels
        """
        center_pos = np.array(position)
        pos = center_pos - np.array((wheelbase/2, 0))
        super().__init__(pos, np.pi, velocity, size)

        self.wheelbase = wheelbase
        # distance from wheel to top/rear car
        self.wheel_to_bound = (self.size[0] - self.wheelbase) / 2
        self.turning_wheel_angle = turning_wheel_angle
        self.turning_radius = self.wheelbase / np.sin(self.turning_wheel_angle)

        self.backwheels_center_to_center = self.wheelbase / np.tan(self.turning_wheel_angle)
        # center of the turning circle
        self.turning_center = np.array((
            self.position[0] + wheelbase,
            self.position[1] + self.backwheels_center_to_center
        ))

    def get_center(self):
        """
        Since center point between two front wheels is maintained for self.position,
        the center point of vehicle shape must be different from self.position
        """
        rot = np.array([
            [np.cos(self.heading), -np.sin(self.heading)],
            [np.sin(self.heading), np.cos(self.heading)]
        ])
        center_offset = np.array((-self.wheelbase/2, 0))
        return self.position + rot @ center_offset

class UTurnSimulation(Simulation):
    """
    Simulation for U-turn scenarios
    """
    def __init__(self, ego: UTurnEgo, npc: UTurnNPC, sim_step=0.02):
        super().__init__(ego, npc, sim_step)

    def npc_step(self):
        super().npc_step()
        delta_s = self.npc.speed * self.sim_step
        # if U-turn finished
        if self.npc.heading == 0:
            self.npc.position[0] += delta_s
        else:
            delta_angle = delta_s / self.npc.turning_radius

            self.npc.position = utils.rotate_point(self.npc.position, self.npc.turning_center, -delta_angle)
            self.npc.heading = max(self.npc.heading - delta_angle, 0)

    def step(self):
        if self.brake_decision_time < 0 and self.should_detect_risk():
            self.brake_decision_time = self.time + RISK_EVAL_TIME

        elif not self.AEB_activated and self.should_activate_AEB():
            self.AEB_activated = True

        super().step()

    # Ego should detect a potential risk
    def should_detect_risk(self):
        return self.npc.topright()[1] >= env_config['lane_width'] / 2 + env_config['median_strip']

    def should_activate_AEB(self):
        return False

def single_sim_exec(dx0, ve, vo, turning_wheel_angle=TURNING_WHEEL_ANGLE,
                    wheelbase=WHEEL_BASE, rightmost_lane=True):
    sim_step = 0.02
    average_length = (env_config['ego_length'] + env_config['npc_length']) / 2

    npc = UTurnNPC((dx0 + average_length, 0),
                   (vo,0),
                   (env_config['npc_length'], env_config['npc_width']),
                    wheelbase, turning_wheel_angle)

    ego_dy0 = env_config['median_strip'] + env_config['lane_width']
    if not rightmost_lane:
        ego_dy0 += env_config['lane_width']
    ego = UTurnEgo((0, ego_dy0),
                   (ve,0),
                   (env_config['ego_length'], env_config['ego_width']))

    sim = UTurnSimulation(ego, npc, sim_step)
    while sim.time < 15:
        sim.step()
        if sim.collision:
            return False
    return True

def simulation(vo, rightmost_lane=True):
    """
    :param vo: NPC speed in m/s
    """

    # red points: collisions
    fc_x, fc_y = [], []

    # green points: no collisions
    nc_x, nc_y = [], []

    for ve in [14,20,25,30,35,40,45,50]:
        for dx in range(9, 51):
            not_collision = single_sim_exec(dx, ve/3.6, vo, rightmost_lane=rightmost_lane)
            if not_collision:
                nc_x.append(dx), nc_y.append(ve)
            else:
                fc_x.append(dx), fc_y.append(ve)

        print(f"Done ve = {ve}")

    # draw config
    shape = ","
    colors = ['r', 'g', 'orange']

    plt.figure(dpi=200, figsize=(10,4.0))

    # plotting points as a scatter plot
    plt.scatter(fc_x, fc_y, label="Collision", color=colors[0], marker=shape, s=20)
    plt.scatter(nc_x, nc_y, label="No collision", color=colors[1], marker=shape, s=20)

    # x-axis label
    plt.xlabel('Longitudinal distance (dx0)')
    plt.ylabel('Ego speed (ve)')
    # plot title
    plt.title(f'Ego: {"rightmost lane" if rightmost_lane else "adjacent lane"}, '
              f'vo = {(int)(vo * 3.6)}')
    # showing legend
    plt.legend(bbox_to_anchor=(0.82, 0.8))
    plt.show()

def cli_parser():
    parser = argparse.ArgumentParser(description='Simulation to Construct '
                                                 'Safety reference benchmark for U-turn scenarios.')
    parser.add_argument('-vo', type=int, default=10,
                      help='NPC Speed in km/h (default: 10)')
    parser.add_argument('-l', '--lane', default="rightmost",
                      help='either `rightmost` or `adjacent` (default: rightmost)')
    return parser

if __name__ == '__main__':
    cli_args = cli_parser().parse_args()
    vo = cli_args.vo / 3.6

    rightmost = cli_args.lane == "rightmost"
    if cli_args.lane not in ["rightmost", "adjacent"]:
        print("[WARNING] Lane must be either `rightmost` or `adjacent`. "
              "Rightmost is used by default")
        rightmost = True
    simulation(vo, rightmost)
