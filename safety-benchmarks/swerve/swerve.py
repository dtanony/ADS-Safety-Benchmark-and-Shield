from common import *
from matplotlib import pyplot as plt

LANE_WIDTH = 3.5
EXCEEDING_Y = 1.0
SWERVE_DISTANCE = 2.0
# assume a compact car
WHEEL_BASE = 2.5

npc_length, npc_width = 4.0, 1.9
ego_length, ego_width = 4.8, 2.0

class SwerveEgo(Ego):
    def __init__(self, position, velocity, size=(2,5)):
        super().__init__(position, 0, velocity, size)

class SwerveNPC(NPC):
    def __init__(self, position, vo, vy, size,
                 exceeding_y=EXCEEDING_Y, swerve_distance=SWERVE_DISTANCE,
                 wheelbase=WHEEL_BASE):
        """
        :param position: center point of the car shape rectangle.
        :param vo:
        :param vy:
        :param size:
        :param exceeding_y:
        :param swerve_distance:
        Note that we maintain self.position as the center point between two back wheels
        """
        vx = np.sqrt(vo**2 - vy**2)
        center_pos = np.array(position)
        pos = center_pos + np.array((wheelbase / 2, 0))

        super().__init__(pos, np.pi, (vx,vy), size)
        self.speed = vo
        self.vx = vx
        self.vy = vy
        self.exceeding_y = exceeding_y
        self.swerve_distance = swerve_distance
        self.wheelbase = wheelbase
        self.angular_speed = 0.0

        # distance from wheel to top/rear car
        self.wheel_to_bound = (self.size[0] - self.wheelbase) / 2

        self.waypoints = []
        ny = LANE_WIDTH / 2 + self.exceeding_y - self.size[1] / 2
        self.waypoints.append(np.array(
            (self.position[0] - vx / vy * ny - self.wheel_to_bound - self.wheelbase, ny)
        ))
        self.waypoints.append(self.waypoints[0] - np.array(
            (self.swerve_distance, 0)
        ))
        self.waypoints.append(self.waypoints[1] - np.array(
            (vx / vy * ny, ny)
        ))
        # dummy waypoint
        self.waypoints.append(self.waypoints[2] - np.array(
            (10, 0)
        ))
        self.wpid = 0

    def forward(self):
        return np.array(
            (np.cos(self.heading), np.sin(self.heading))
        )

    def get_center(self):
        """
        Since center point between two rear wheels is maintained for self.position,
        the center point of vehicle shape must be different from self.position
        """
        rot = np.array([
            [np.cos(self.heading), -np.sin(self.heading)],
            [np.sin(self.heading), np.cos(self.heading)]
        ])
        center_offset = np.array((self.wheelbase/2, 0))
        return self.position + rot @ center_offset

    def front_center(self):
        local_front_center = np.array(
            [self.wheel_to_bound + self.wheelbase, 0]
        )
        rot = np.array([
            [np.cos(self.heading), -np.sin(self.heading)],
            [np.sin(self.heading), np.cos(self.heading)]
        ])
        return (rot @ local_front_center.T).T + self.position

class SwerveSimulation(Simulation):
    """
    Simulation for swerve scenarios
    """
    def __init__(self, ego: SwerveEgo, npc: SwerveNPC, sim_step=0.02):
        super().__init__(ego, npc, sim_step)
        self.npc_delta_s = self.npc.speed * self.sim_step

    def npc_step(self):
        super().npc_step()
        if self.npc.wpid > 3:
            return
        dis_to_waypoint = np.linalg.norm(self.npc.waypoints[self.npc.wpid] - self.npc.front_center())
        if dis_to_waypoint <= 1.5 * self.sim_step * self.npc.speed:
            # update waypoint
            self.npc.wpid += 1
            if self.npc.wpid > 3:
                return

        self.update_npc_pose()
        self.update_npc_angular_speed()

    def update_npc_angular_speed(self):
        target_point = self.npc.waypoints[self.npc.wpid]
        steering_direction = target_point - self.npc.position
        steering_angle = utils.signed_angle_2d(self.npc.forward(), steering_direction)

        lookahead = np.linalg.norm(target_point - self.npc.front_center())
        if lookahead < 1e-6:
            return
        target_yaw_speed = 2 * self.npc.speed * np.sin(steering_angle) / lookahead
        self.npc.angular_speed = target_yaw_speed

    def update_npc_pose(self):
        self.npc.heading += self.npc.angular_speed * self.sim_step
        self.npc.position += self.npc.forward() * (self.npc.speed * self.sim_step)

    def step(self):
        if self.brake_decision_time < 0 and self.should_detect_risk():
            self.brake_decision_time = self.time + RISK_EVAL_TIME

        elif not self.AEB_activated and self.should_activate_AEB():
            self.AEB_activated = True

        super().step()

    # Ego should detect a potential risk
    def should_detect_risk(self):
        return self.npc.topright()[1] >= LANE_WIDTH / 2

    def should_activate_AEB(self):
        return False

def single_sim_exec(dx0, ve, vo,vy, exceeding_y,swerve_distance):
    sim_step = 0.025
    average_length = (ego_length + npc_length) / 2.0

    npc = SwerveNPC((dx0 + average_length, 0.0),
                    vo, vy,
                    (npc_length, npc_width),
                    exceeding_y,swerve_distance)

    ego = SwerveEgo((0.0, LANE_WIDTH),
                    (ve,0.0),
                    (ego_length, ego_width))

    sim = SwerveSimulation(ego, npc, sim_step)
    while sim.time < 10:
        sim.step()
        if sim.collision:
            return False
    return True

def simulation():
    ve = 40 / 3.6
    vo = 15 / 3.6
    exceeding_y = EXCEEDING_Y
    swerve_distance = SWERVE_DISTANCE

    # red points: collisions
    fc_x, fc_y = [], []

    # green points: no collisions
    nc_x, nc_y = [], []

    vy = 0.6
    while vy <= 1.61:
        for dx in range(15, 56):
            not_collision = single_sim_exec(dx, ve, vo,vy, exceeding_y, swerve_distance)
            if not_collision:
                nc_x.append(dx), nc_y.append(vy)
            else:
                fc_x.append(dx), fc_y.append(vy)

        print(f"Done vy = {vy:.2f}")
        vy += 0.1

    # draw config
    shape = ","
    colors = ['r', 'g', 'orange']

    plt.figure(dpi=300, figsize=(10,4))

    # plotting points as a scatter plot
    plt.scatter(fc_x, fc_y, label="collision", color=colors[0], marker=shape, s=20)
    plt.scatter(nc_x, nc_y, label="no collision", color=colors[1], marker=shape, s=20)

    # x-axis label
    plt.xlabel('Longitudinal distance (dx0)')
    plt.ylabel('Lateral velocity (vy)')
    # plot title
    plt.title(f've = {(int)(ve * 3.6)}, vo = {(int)(vo * 3.6)}, '
              f'exceed_y = {exceeding_y}, '
              f'swerve_dis = {swerve_distance}')
    # showing legend
    plt.legend(bbox_to_anchor=(0.8, 0.9))
    plt.show()

if __name__ == '__main__':
    simulation()