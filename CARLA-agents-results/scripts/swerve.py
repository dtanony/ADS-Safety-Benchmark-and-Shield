import carla
import time, math, json
from bench_common import *
from PCLA import PCLA

dx0_map = {
    '10': {  # vo
        '1.0': 18.0,  # vy
        '1.2': 16.0,
        '1.4': 15.0,
    },
    '15': {
        '1.0': 23.0,
        '1.2': 20.0,
        '1.4': 17.0,
    }
}

dt = 0.05

def d_dx0(vo, vy):
    if dx0_map[str(int(vo))][str(vy)]:
        return dx0_map[str(int(vo))][str(vy)]
    print(f"Warning: no predefined dx0 for vo={vo}, vy={vy}. Using default dx0=16.0")
    return 16.0

def make_pid(vo):
    if vo >= 15.0:
        return SpeedPID(kp=0.4, ki=0.15, kd=0.0)
    return SpeedPID(kp=0.5, ki=0.2, kd=0.0)

def trigger_move(ego_transform, npc, vo):
    if vo == 10 and ego_transform.location.distance(npc.get_transform().location) < 55.0:
        return True
    elif vo == 15 and ego_transform.location.distance(npc.get_transform().location) < 65.0:
        return True
    return False

def trigger_end(ego_transform, npc, time_acc):
    if time_acc > 15 and \
       ego_transform.get_forward_vector().dot(npc.get_transform().location - ego_transform.location) < 0 and\
       npc.get_transform().location.distance(ego_transform.location) > 40:
        return True
    if time_acc >= 60:
        return True
    return False

def make_swerve(world, map_, vehicle, swerve_vy, 
                swerve_ny=1.8, swerve_dis=2.0, swerve_left=True):
    transform = vehicle.get_transform()
    npc_speed = get_speed(vehicle)
    forward = transform.get_forward_vector()
    front_center = get_front_center(vehicle)
    forward_np_2d = carla_vector3d_to_numpy(forward, _2d=True)

    angle = math.asin(swerve_vy / npc_speed)
    x_shift = swerve_ny / math.tan(angle)

    wp1 = point_forward(carla_vector3d_to_numpy(front_center, _2d=True),
                        forward_np_2d,
                        x_shift,
                        swerve_ny if swerve_left else -swerve_ny)

    wp2 = wp1 + forward_np_2d * swerve_dis
    wp3 = point_forward(wp2,
                        forward_np_2d,
                        x_shift,
                        -swerve_ny if swerve_left else swerve_ny)

    # Make swerve waypoints
    waypoints = []
    for wp in [wp1, wp2, wp3]:
        waypoints.append(carla.Location(x=wp[0], y=wp[1], z=front_center.z))
    return waypoints

# target_speed in m/s
def control_npc(world, map_, ego, npc, pid, target_speed, swerve_done, next_waypoints, next_waypoint, dx0, vy):
    meta_waypoints = None
    # Current state
    transform = npc.get_transform()
    front_center = get_front_center(npc)
    forward = transform.get_forward_vector()

    if longitudinal_distance(ego, npc) <= dx0 and not swerve_done:
        swerve_done = True
        updated_waypoints = make_swerve(world, map_, npc, vy)
        # plot_points(world, updated_waypoints, color=carla.Color(0, 255, 0), z_offset=0.1)
        meta_waypoints = [front_center] + updated_waypoints

        next_waypoints = updated_waypoints
        next_waypoint = next_waypoints.pop(0)

    if hasattr(next_waypoint, 'transform'):
        target_loc = next_waypoint.transform.location
    else:
        target_loc = next_waypoint

    # Apply control
    control = make_carla_control(npc, target_loc, pid, target_speed, dt)
    npc.apply_control(control)

    next_waypoints, next_waypoint = update_waypoints(next_waypoints, next_waypoint, front_center, forward, world, map_)
    return swerve_done, next_waypoints, next_waypoint, meta_waypoints

# vo in km/h
def run_one_agent(args, world, map_, client, route, vehicle_spawn_points, pid, vo, vy, dx0, bp_library):
    # Spawn ego vehicle
    ego_bp = bp_library.find('vehicle.toyota.prius')
    ego_bp.set_attribute('role_name', 'hero')

    ego_start_pos = vehicle_spawn_points[1]
    ego = world.spawn_actor(ego_bp, ego_start_pos)

    # Spawn NPC vehicle
    npc_bp = bp_library.find('vehicle.audi.a2')

    npc_start_pos = vehicle_spawn_points[84]
    current_waypoint = map_.get_waypoint(npc_start_pos.location)
    next_waypoints = current_waypoint.next_until_lane_end(10.0)
    next_waypoint = next_waypoints.pop(0)
    if len(next_waypoints) == 0:
        next_waypoints.extend(make_wp(next_waypoint, map_))

    npc = world.spawn_actor(npc_bp, npc_start_pos)
    # plot_points(world, next_waypoints)     # for debug

    # Set spectator to top-down view
    spectator = world.get_spectator()
    spectator.set_transform(viewpoint_transform(ego_start_pos))
    world.tick()

    data = {
        'groundtruth_kinematic': [],
        'groundtruth_size': [
            write_shape_info(ego.bounding_box, 'ego'),
            write_shape_info(npc.bounding_box, 'npc1')
        ],
        'metadata': { }
    }
    time_acc = 0.0
    swerve_done = False
    moving = False
    try:
        pcla = PCLA(args.agent, ego, route, client)
        while True:
            data['groundtruth_kinematic'].append(write_info(data, time_acc, ego, npc))
            time_acc += dt

            ego_transform = ego.get_transform()
            spectator.set_transform(viewpoint_transform(ego_transform))
        
            # Ego
            ego_action = pcla.get_action()
            ego.apply_control(ego_action)
            print(f"Ego speed: {get_speed(ego)*3.6:.2f}, NPC speed: {get_speed(npc)*3.6:.2f} km/h")

            # NPC
            if trigger_move(ego_transform, npc, vo) and not moving:
                moving = True
                print("NPC starts moving")
            if moving:
                swerve_done, next_waypoints, next_waypoint, meta_waypoints = control_npc(
                    world, map_, ego, npc, pid, vo/3.6, swerve_done, next_waypoints, next_waypoint, dx0, vy)
                if meta_waypoints is not None:
                    data['metadata']['waypoints'] = [write_vector3d(p) for p in meta_waypoints]

            world.tick()

            if trigger_end(ego_transform, npc, time_acc):
                print("Ending scenario.")
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("Saving and exiting...")
        client.stop_recorder()
        # save to json file
        with open(args.output + f"_{args.agent}_{int(vo)}_{int(vy*10)}.json", 'w') as f:
            json.dump(data, f, indent=None)

        # Full stop
        npc.apply_control(carla.VehicleControl(brake=1.0))
        time.sleep(1.0)

        # Cleanup
        npc.destroy()
        ego.destroy()
        pcla.cleanup()
        time.sleep(1.0)

def main(args):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    client.load_world("Town07")
    world = client.get_world()
    map_ = world.get_map()
    traffic_manager = client.get_trafficmanager(8000)
    settings = world.get_settings()
    asynch = False
    if not asynch:
        traffic_manager.set_synchronous_mode(True)
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = dt
    else:
        print("You are currently in asynchronous mode. If this is a traffic simulation, \
                you could experience some issues. If it's not working correctly, switch to \
                synchronous mode by using traffic_manager.set_synchronous_mode(True)")
    world.apply_settings(settings)
    
    bp_library = world.get_blueprint_library()
    vehicle_spawn_points = map_.get_spawn_points()
    
    route = "./route-1-83.xml"
    for vo in [10, 15]:
        for vy in [1.0, 1.2, 1.4]:
            dx0 = d_dx0(vo, vy)
            pid = make_pid(vo)
            print(f"Running agent: {args.agent} with vo={vo}, vy={vy}, dx0={dx0}")
            client.start_recorder(args.output + f"_{args.agent}_{vo}_{int(vy*10)}.log", True)
            run_one_agent(args, world, map_, client, route, vehicle_spawn_points, pid, vo, vy, dx0, bp_library)

def make_cli_args():
    import argparse
    parser = argparse.ArgumentParser(description="CARLA Swerve Scenario")
    parser.add_argument('output', type=str, help='Output file for saving the (replayable) log and trace data. Must be an absolute path.')
    parser.add_argument('-a', '--agent', type=str, help='Agent name, either "tf_tf", "lav_lav", "if_if", "tf_ltf", "tf_gf", "tf_lf"', default='tf_tf')
    return parser.parse_args()

if __name__ == '__main__':
    main(make_cli_args())