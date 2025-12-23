import carla
import time, math
import numpy as np

class SpeedPID:
    def __init__(self, kp=0.4, ki=0.05, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, target_speed, current_speed, dt):
        error = target_speed - current_speed
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        return output

def make_carla_control(npc, target_loc, pid, target_speed, dt=0.05):
    position = get_middle_rear_wheels_position(npc)
    speed = get_speed(npc)
    transform = npc.get_transform()
    
    # vector from vehicle front center to target in world frame
    dx = target_loc.x - position.x
    dy = target_loc.y - position.y
    Ld = math.hypot(dx, dy)

    steer = 0.0
    if Ld > 1e-3:
        # heading to target and current yaw (radians)
        angle_to_target = math.atan2(dy, dx)
        yaw_rad = math.radians(transform.rotation.yaw)
        # normalize heading error to [-pi, pi]
        alpha = angle_to_target - yaw_rad
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # pure pursuit curvature
        wheelbase = 2.5
        curvature = 2.0 * math.sin(alpha) / max(Ld, 1e-6)
        steer_angle = math.atan(wheelbase * curvature)  # radians
        # normalize to [-1,1] using an approximate max steer (rad)
        max_steer_angle = math.pi/6
        steer = max(-1.0, min(1.0, steer_angle / max_steer_angle))

    # Apply control
    control = carla.VehicleControl()
    control.steer = steer
    u = pid.step(target_speed, speed, dt)
    if u >= 0:
        control.throttle = min(u, 1.0)
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = min(-u, 1.0)
    return control

def make_wp(input, map_):
    if isinstance(input, carla.Vector3D):
        input = map_.get_waypoint(input)
    wp = input.next(5.0)[0]
    new_waypoints = wp.next_until_lane_end(10.0)
    new_wps = [wp] + new_waypoints
    return new_wps

def update_waypoints(next_waypoints, next_waypoint, front_center, forward, world, map_):
    # Check to update the next waypoint
    if forward.dot(wp_location(next_waypoint) - front_center) <= 0.0:
        next_waypoint = next_waypoints.pop(0)
        if len(next_waypoints) < 2:
            last_wp = next_waypoints[-1]
            new_wps = make_wp(last_wp, map_)
            # plot_points(world, new_wps)
            next_waypoints.extend(new_wps)

    return next_waypoints, next_waypoint
    
def rotate_point(point, center, angle_radians):
    """
    Rotates a 2D point around a given center point.
    :param point: np.array
    :param center: np.array
    :param angle_degrees:
    :return:
    """
    if isinstance(point, carla.Vector3D):
        point = np.array([point.x, point.y])
    if isinstance(center, carla.Vector3D):
        center = np.array([center.x, center.y])
    translated_point = point - center
    rotation_matrix = np.array([
        [math.cos(angle_radians), -math.sin(angle_radians)],
        [math.sin(angle_radians),  math.cos(angle_radians)]
    ])
    rotated_translated_point = np.dot(rotation_matrix, translated_point)
    return rotated_translated_point + center

def point_forward(base_position, forward_direction, x_shift, y_shift):
    """
    Get a point from the base position and forward direction with x_shift and y_shift.
    :param base_position: np array of size 2
    :param forward_direction: normalized direction vector. np array of size 2
    :param x_shift: longitudinal shift, should not be negative
    :param y_shift: lateral shift. Positive if right shift, negative if left shift.
    :return:
    """
    dis = np.sqrt(x_shift ** 2 + y_shift ** 2)
    fwd_point = base_position + forward_direction * dis
    angle = np.arctan2(np.abs(y_shift), x_shift)
    if y_shift > 0:
        angle = -angle
    return rotate_point(fwd_point, base_position, angle)

def get_speed(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2)

def get_front_center(vehicle):
    bbox = vehicle.bounding_box
    tf = vehicle.get_transform()
    # Front center in vehicle local frame
    front_local = carla.Location(
        x=bbox.location.x + bbox.extent.x,
        y=bbox.location.y,
        z=bbox.location.z
    )
    # Convert to world frame
    front_world = tf.transform(front_local)
    return front_world

def viewpoint_transform(ref_transform):
    spectator_location = ref_transform.location - ref_transform.get_forward_vector()*6.0 + carla.Location(z=4.0)
    return carla.Transform(spectator_location, carla.Rotation(pitch=-15.0,yaw=ref_transform.rotation.yaw))

def topdown_viewpoint(ref_transform, z_offset=50.0):
    loc = ref_transform.location
    top_down_tf = carla.Transform(
        location=carla.Location(x=loc.x, y=loc.y, z=z_offset),
        rotation=carla.Rotation(pitch=-90.0, yaw=ref_transform.rotation.yaw)
    )
    return top_down_tf

def longitudinal_distance(ego, npc):
    ego_front_center = get_front_center(ego)
    npc_front_center = get_front_center(npc)
    ego_forward = ego.get_transform().get_forward_vector()
    return ego_forward.dot(npc_front_center - ego_front_center)

def write_vector3d(vec):
    return {
        'x': vec.x,
        'y': vec.y,
        'z': vec.z
    }
def write_kinematic(transform, velocity):
    return {
        'pose': {
            'position': write_vector3d(transform.location),
            'rotation': {
                'x': transform.rotation.roll,
                'y': transform.rotation.pitch,
                'z': transform.rotation.yaw
            }
        },
        'twist': {
            'linear': write_vector3d(velocity)
        }
    }

def write_info(data, timestamp, ego, npc):
    ego_transform = ego.get_transform()
    npc_transform = npc.get_transform()

    ego_entry = write_kinematic(ego_transform, ego.get_velocity())
    ego_entry['acceleration'] = {}
    ego_entry['acceleration']['linear'] = write_vector3d(ego.get_acceleration())

    npc_entry = write_kinematic(npc_transform, npc.get_velocity())
    npc_entry['name'] = "NPC"

    return {
        'timestamp': timestamp,
        'groundtruth_ego': ego_entry,
        'groundtruth_vehicles': [npc_entry]
    }

def write_shape_info(bounding_box, name):
    return {
        'name': name,
        'center': write_vector3d(bounding_box.location),
        'size': write_vector3d(bounding_box.extent * 2)
    }

def plot_points(world, points, color=carla.Color(255, 0, 0), size=0.1, life_time=6.0, z_offset=1.0):
    for p in points:
        pos = p
        if isinstance(p, carla.Waypoint):
            pos = p.transform.location
        world.debug.draw_point(
            pos + carla.Location(z=z_offset),                      # carla.Location
            size=size,               # radius (meters)
            color=color,
            life_time=life_time           # 0 = persistent
        )

def wp_location(waypoint):
    if isinstance(waypoint, carla.Vector3D):
        return waypoint
    if isinstance(waypoint, carla.Waypoint):
        return waypoint.transform.location

def get_middle_rear_wheels_position(vehicle):
    physics_control = vehicle.get_physics_control()
    wheels = physics_control.wheels
    rear_center_wheels = (wheels[2].position + wheels[3].position) / 200
    return rear_center_wheels

def update_yaw_speed(vehicle, next_waypoint):
    speed = get_speed(vehicle)
    transform = vehicle.get_transform()
    # position = transform.location
    position = get_middle_rear_wheels_position(vehicle)
    front_center = get_front_center(vehicle)
    forward = transform.get_forward_vector()

    next_wp_location = wp_location(next_waypoint)
    lookahead_dis = front_center.distance(next_wp_location)
    direction = next_wp_location - position
    direction.z = 0.0

    forward.z = 0.0
    cross = forward.x * direction.y - forward.y * direction.x
    dot   = forward.x * direction.x + forward.y * direction.y
    signed_angle = math.atan2(cross, dot)
    yaw_speed = 2 * speed * math.sin(signed_angle) / max(lookahead_dis, 0.03)
    return yaw_speed

def carla_vector3d_to_numpy(vec, _2d=False):
    if _2d:
        return np.array([vec.x, vec.y])
    return np.array([vec.x, vec.y, vec.z])
