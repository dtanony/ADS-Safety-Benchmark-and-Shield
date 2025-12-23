from sys import argv
import carla

def make_cli_args():
    import argparse
    parser = argparse.ArgumentParser(description="CARLA Replay Viewer")
    parser.add_argument('input', type=str, help='Input replay file (e.g., input.rec)')
    parser.add_argument('-f', '--time-factor', type=float, help='Time factor for replay speed', default=1.0)
    parser.add_argument('-s', '--start', type=float, help='Start time for replay', default=0.0)
    parser.add_argument('-d', '--duration', type=float, help='Duration for replay', default=0)
    return parser.parse_args()

if __name__ == '__main__':
    args = make_cli_args()
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        client.load_world("Town10HD")
        print(client.show_recorder_file_info(args.input, False))

        world = client.get_world()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        
        client.set_replayer_time_factor(args.time_factor)
        client.replay_file(args.input, args.start, args.duration, 0)

        spectator = world.get_spectator()
        ego = None
        ttc = 1e9

        while True:
            world.tick()

            if ego is None: 
                vehicles = world.get_actors().filter('vehicle.*')
                ego = next((v for v in vehicles if v.attributes.get('role_name') == 'hero'), None)
            else:
                ego_transform = ego.get_transform()
                spectator_location = ego_transform.location - ego_transform.get_forward_vector()*6.0 + carla.Location(z=4.0)
                spectator_transform = carla.Transform(spectator_location, carla.Rotation(pitch=-15.0,yaw=ego_transform.rotation.yaw))
                spectator.set_transform(spectator_transform)

        # client.set_replayer_time_factor(1)
    except KeyboardInterrupt:
        pass
    finally:
        #settings.no_rendering_mode = False
        settings.synchronous_mode = False
        world.apply_settings(settings)
