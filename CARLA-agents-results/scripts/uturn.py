import carla
import time, math, json
from bench_common import *
from PCLA import PCLA

dx0_map = {
    'innermost': {
        '10': 12.0,
        '15': 10.0,
    },
    'adjacent': {
        '10': 12.0,
        '15': 10.0,
    }
}
dt = 0.05

def d_dx0(lane, vo):
    if dx0_map[lane][str(int(vo))]:
        return dx0_map[lane][str(int(vo))]
    print(f"Warning: no predefined dx0 for lane={lane}, vo={vo}. Using default dx0=12.0")
    return 12.0

def get_route(lane):
    if lane == 'innermost':
        return "./route-15-39.xml"
    elif lane == 'adjacent':
        return "./route-71-108.xml"
    else:
        raise ValueError(f"Unknown lane: {lane}")

def make_pid(vo):
    if vo >= 15.0:
        return SpeedPID(kp=0.5, ki=0.2, kd=0.0)
    return SpeedPID(kp=0.6, ki=0.3, kd=0.0)

def trigger_end(ego, ego_transform, npc, time_acc):
    npc_transform = npc.get_transform()
    if time_acc > 15 and \
       ego_transform.get_forward_vector().dot(npc_transform.get_forward_vector()) > 0.9:
        if ego_transform.location.distance(npc_transform.location) > 15:
            return True
    if time_acc >= 40:
        return True
    return False

def make_uturn(world, map_, vehicle):
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()
    # forward vector be rotated 90 degrees counter-clockwise
    left_direction = carla.Vector3D(forward.y, -forward.x, 0.0)

    front_center = get_front_center(vehicle)
    current_fc_wp = map_.get_waypoint(front_center)
    radius = max(current_fc_wp.lane_width, 3.8)

    rear_center_wheels = get_middle_rear_wheels_position(vehicle)
    uturn_center = rear_center_wheels + left_direction * radius

    # Find waypoints along the U-turn path
    waypoints = []
    for i in range(1, 4):
        wp = rotate_point(front_center, uturn_center, -i*math.pi/3)
        waypoints.append(carla.Location(x=wp[0], y=wp[1], z=front_center.z))

    return waypoints

def control_npc(world, map_, ego, npc, dt, pid, target_speed, uturn_done, next_waypoints, next_waypoint, dx0=12.0):
    meta_waypoints = None
    # Current state
    transform = npc.get_transform()
    front_center = get_front_center(npc)
    forward = transform.get_forward_vector()
    
    if longitudinal_distance(ego, npc) <= dx0 and not uturn_done:
        # make a U-turn
        uturn_done = True
        updated_waypoints = make_uturn(world, map_, npc)
        # plot_points(world, updated_waypoints, color=carla.Color(0, 255, 0))
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
    return uturn_done, next_waypoints, next_waypoint, meta_waypoints

# vo in km/h
def run_one_agent(args, world, map_, client, vehicle_spawn_points, pid, lane, vo, dx0, bp_library):
    # Spawn ego vehicle
    ego_bp = bp_library.find('vehicle.toyota.prius')
    ego_bp.set_attribute('role_name', 'hero')
    ego_start_pos = vehicle_spawn_points[15 if lane == 'innermost' else 71]
    ego = world.spawn_actor(ego_bp, ego_start_pos)

    # Spawn NPC vehicle
    npc_bp = bp_library.find('vehicle.audi.a2')
    if lane == 'innermost':
        wp_20 = map_.get_waypoint(vehicle_spawn_points[20].location)
        prev_wp = wp_20.previous(10.0)[0]
    else:
        wp_67 = map_.get_waypoint(vehicle_spawn_points[67].location)
        prev_wp = wp_67.previous(10.0)[0]

    next_waypoints = prev_wp.next_until_lane_end(10.0)
    next_waypoint = next_waypoints.pop(0)
    npc = world.spawn_actor(npc_bp, prev_wp.transform)
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
    uturn_done = False
    moving = False
    try:
        route = get_route(lane)
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
            if get_speed(ego) >= 13.8/3.6 and not moving and ego_transform.location.distance(npc.get_transform().location) < 80.0:
                moving = True
                print("NPC starts moving")
            if moving:
                uturn_done, next_waypoints, next_waypoint, meta_waypoints = control_npc(
                    world, map_, ego, npc, dt, pid, vo/3.6, uturn_done, next_waypoints, next_waypoint, dx0)
                if meta_waypoints is not None:
                    data['metadata']['waypoints'] = [write_vector3d(p) for p in meta_waypoints]

            world.tick()

            if trigger_end(ego, ego_transform, npc, time_acc):
                print("Ending scenario.")
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("Saving and exiting...")
        client.stop_recorder()
        # save to json file
        with open(args.output + f"_{args.agent}_{lane}_{int(vo)}.json", 'w') as f:
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
    client.load_world("Town10HD")
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
    
    for lane in ['innermost', 'adjacent']:
        for vo in [10, 15]:
            dx0 = d_dx0(lane, vo)
            pid = make_pid(vo)
            print(f"Running agent: {args.agent} with lane={lane}, vo={vo}, dx0={dx0}")
            client.start_recorder(args.output + f"_{args.agent}_{lane}_{int(vo)}.log", True)
            run_one_agent(args, world, map_, client, vehicle_spawn_points, pid, lane, vo, dx0, bp_library)

def make_cli_args():
    import argparse
    parser = argparse.ArgumentParser(description="CARLA U-turn Scenario")
    parser.add_argument('output', type=str, help='Output file for saving the (replayable) log and trace data. Must be an absolute path.')
    parser.add_argument('-a', '--agent', type=str, help='Agent name, either "tf_tf", "lav_lav", "if_if", "tf_ltf", "tf_gf", "tf_lf"', default='tf_tf')
    return parser.parse_args()

if __name__ == '__main__':
    main(make_cli_args())