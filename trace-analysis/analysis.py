import json
import math
import numpy as np
from vehicle import Vehicle
import os, sys, re
import utils
from pathlib import Path

UTURN_KEY_STR = "uturn_point"
UTURN_FILE_PATTERN = r"uturn_sim\d+\.json"

SWERVE_KEY_STR = "swerve_point"
SWERVE_FILE_PATTERN = r"swerve_sim\d+\.json"

def is_moving(twist_linear, threshold=1e-3):
    # Compute the magnitude of the velocity vector
    v = math.sqrt(twist_linear['x']**2 + twist_linear['y']**2 + twist_linear['z']**2)
    return v > threshold

def load_data(filepath):
    with open(filepath, 'r') as f:
        return json.load(f)
    
def npc_starts_moving_moment(data):
    for entry in data['groundtruth_kinematic']:
        vehicles = entry.get('groundtruth_vehicles', [])
        if not vehicles:
            continue
        npc = vehicles[0]  # Assuming only one NPC
        twist_linear = npc['twist']['linear']
        if is_moving(twist_linear):
            return entry['timestamp']
    return None

def kinematics_at(timestamp, data):
    for entry in data['groundtruth_kinematic']:
        if entry['timestamp'] == timestamp:
            ego_kin = entry['groundtruth_ego']
            npc_kin = entry['groundtruth_vehicles'][0]
            return ego_kin, npc_kin
    return None

def position_at(timestamp, data):
    kinematics = kinematics_at(timestamp, data)
    if kinematics:
        ego_kin, npc_kin = kinematics
        ego_pos = np.array([ego_kin['pose']['position']['x'],
                            ego_kin['pose']['position']['y'],
                            ego_kin['pose']['position']['z']])
        npc_pos = np.array([npc_kin['pose']['position']['x'],
                            npc_kin['pose']['position']['y'],
                            npc_kin['pose']['position']['z']])
        return ego_pos, npc_pos
    return None

def distance_at(timestamp, data):
    positions = position_at(timestamp, data)
    if positions:
        ego_pos, npc_pos = positions
        return np.linalg.norm(ego_pos - npc_pos)
    return None

def longitudinal_distance_at(timestamp, data):
    veh_details = extract_vehicle_sizes(data)
    npc_details = next(item for item in veh_details if item['name'] == 'npc1')
    npc_length = npc_details['size']['x']
    npc_center_x = npc_details['center']['x']

    ego_details = next(item for item in veh_details if item['name'] == 'ego')
    ego_length = ego_details['size']['x']
    ego_center_x = ego_details['center']['x']

    kinematics = kinematics_at(timestamp, data)
    if kinematics:
        ego_kin, npc_kin = kinematics
        ego_pos = np.array([ego_kin['pose']['position']['x'],
                            ego_kin['pose']['position']['y']])
        npc_pos = np.array([npc_kin['pose']['position']['x'],
                            npc_kin['pose']['position']['y']])
        ego_heading = math.radians(ego_kin['pose']['rotation']['z'])
        # Ego heading unit vector
        heading_vec = np.array([math.cos(ego_heading), math.sin(ego_heading)])
        # Vector from ego to npc
        delta = npc_pos - ego_pos
        # Project delta onto heading_vec
        longitudinal_dist = np.dot(delta, heading_vec)
        return longitudinal_dist - (ego_length/2 + ego_center_x) - (npc_length/2 + npc_center_x)

    return None

def vehicle(kinematic, size, center):
    pos = np.array([kinematic['pose']['position']['x'],
                    kinematic['pose']['position']['y']])
    heading = kinematic['pose']['rotation']['z']
    return Vehicle(size, pos, heading, center)

def extract_vehicle_sizes(data):    
    veh_sizes = data['groundtruth_size']['vehicle_sizes']
    return veh_sizes

def behavior_start_moment(data, key_str):
    """
    Return the moment when U-turn or Swerve starts
    """
    veh_details = extract_vehicle_sizes(data)
    npc_details = next(item for item in veh_details if item['name'] == 'npc1')
    npc_size = (npc_details['size']['x'], npc_details['size']['y'])
    npc_center = (npc_details['center']['x'], npc_details['center']['y'])

    way_point = np.array((data['metadata'][key_str]['x'],
                   data['metadata'][key_str]['y']))

    re = 0
    min_dis = float('inf')
    saved_entry = None
    for entry in data['groundtruth_kinematic']:
        timestamp = entry['timestamp']
        npc_kin = entry['groundtruth_vehicles'][0]

        npc_heading = math.radians(npc_kin['pose']['rotation']['z'])
        # heading unit vector
        heading_vec = np.array([math.cos(npc_heading), math.sin(npc_heading)])

        npc_veh = vehicle(npc_kin, npc_size, npc_center)
        vec = way_point - npc_veh.get_mid_front()
        sign = np.dot(vec, heading_vec)
        dis = np.linalg.norm(vec)
        if sign < 0:
            break
        if dis < min_dis:
            re = timestamp
            saved_entry = entry
            min_dis = dis
    # print(f'Time U-turn/Swerve starts: {re}')
    return re

def get_speed(ln_vel):
    vel = np.array([
        ln_vel['x'], ln_vel['y'], ln_vel['z']
    ])
    return np.linalg.norm(vel)

def is_collision(data, starting_time=None):
    """
    Check whether a collision exists
    """
    veh_details = extract_vehicle_sizes(data)
    npc_details = next(item for item in veh_details if item['name'] == 'npc1')
    ego_details = next(item for item in veh_details if item['name'] == 'ego')
    npc_size = (npc_details['size']['x'], npc_details['size']['y'])
    npc_center = (npc_details['center']['x'], npc_details['center']['y'])
    ego_size = (ego_details['size']['x'], ego_details['size']['y'])
    ego_center = (ego_details['center']['x'], ego_details['center']['y'])

    for entry in data['groundtruth_kinematic']:
        timestamp = entry['timestamp']
        if starting_time and timestamp < starting_time:
            continue

        npc_kin = entry['groundtruth_vehicles'][0]
        ego_kin = entry['groundtruth_ego']

        ego = vehicle(ego_kin, ego_size, ego_center)
        npc = vehicle(npc_kin, npc_size, npc_center)
        if Vehicle.is_collision(ego, npc):
            return True, timestamp
    return False, -1

def min_ttc(data, starting_time=None):
    """
    Return the minimum TTC between two vehicles.
    If TTC > 3, ignore.
    If A collision occurs, return 0
    """
    veh_details = extract_vehicle_sizes(data)
    npc_details = next(item for item in veh_details if item['name'] == 'npc1')
    ego_details = next(item for item in veh_details if item['name'] == 'ego')
    npc_size = (npc_details['size']['x'], npc_details['size']['y'])
    npc_center = (npc_details['center']['x'], npc_details['center']['y'])
    ego_size = (ego_details['size']['x'], ego_details['size']['y'])
    ego_center = (ego_details['center']['x'], ego_details['center']['y'])

    time_step = 0.01
    time_bound = 3

    ttc = float('inf')
    for entry in data['groundtruth_kinematic']:
        timestamp = entry['timestamp']
        if starting_time and (
                timestamp < starting_time or timestamp > starting_time + 10):
            continue

        npc_kin = entry['groundtruth_vehicles'][0]
        ego_kin = entry['groundtruth_ego']

        ego = vehicle(ego_kin, ego_size, ego_center)
        npc = vehicle(npc_kin, npc_size, npc_center)

        ego_vel = np.array((
            ego_kin['twist']['linear']['x'],
            ego_kin['twist']['linear']['y'],
        ))
        npc_vel = np.array((
            npc_kin['twist']['linear']['x'],
            npc_kin['twist']['linear']['y'],
        ))
        time = 0
        while time < time_bound:
            if Vehicle.is_collision(ego, npc):
                if time < ttc:
                    ttc = time
                break

            ego.advance(ego_vel, time_step)
            npc.advance(npc_vel, time_step)
            time += time_step

    return ttc

def get_lastest_gt_info(data, timestamp):
    last_ego_kin = None
    last_npc_kin = None
    correct_time = -1
    for entry in data['groundtruth_kinematic']:
        if entry['timestamp'] <= timestamp:
            last_ego_kin = entry['groundtruth_ego']
            last_npc_kin = entry['groundtruth_vehicles'][0]
            correct_time = entry['timestamp']
        else:
            break
    return correct_time, last_ego_kin, last_npc_kin

def process_a_file(file_path, file_name=None):
    if not file_name:
        file_name = os.path.basename(file_path)
    data = load_data(file_path)

    if SWERVE_KEY_STR in data['metadata']:
        start_moment = (behavior_start_moment(data, SWERVE_KEY_STR))
    else:
        start_moment = behavior_start_moment(data, UTURN_KEY_STR)

    collision, ti = is_collision(data, start_moment)
    col_str = f"Y ({ti})" if collision else "N"

    minttc = 0
    speed_at_collide = 0
    if collision:
        ego_k,_ = kinematics_at(ti, data)
        speed_at_collide = get_speed(ego_k['twist']['linear'])
    else:
        minttc = min_ttc(data, start_moment)

    dx0 = longitudinal_distance_at(start_moment, data)

    ego_kin, npc_kin = kinematics_at(start_moment, data)
    ego_speed = get_speed(ego_kin['twist']['linear'])
    npc_speed = get_speed(npc_kin['twist']['linear'])

    return file_name, utils.round_float(dx0), \
            utils.round_float(ego_speed), utils.round_float(npc_speed), col_str, minttc, speed_at_collide
    
def process_a_dir(dir_path="../"):
    result = []
    folder = Path(dir_path)
    for file in sorted(folder.iterdir()):
        if file.is_file() and (
            re.fullmatch(SWERVE_FILE_PATTERN, file.name) or
            re.fullmatch(UTURN_FILE_PATTERN, file.name)):
            print(f"Processing file: {file.name}...")
            file_path = os.path.join(dir_path, file.name)
            result.append(process_a_file(file_path, file.name))
    return result

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Trace analysis.")
    parser.add_argument("path", help="Path to a JSON trace file or a folder containing JSON files.")
    parser.add_argument('-u', '--unit', default='m',
                      help='either m (m/s) or km (km/h) (default: m)')
    
    args = parser.parse_args()
    unit = 'm'
    if args.unit == "km":
        unit = "km"
    if os.path.isfile(args.path):
        fn, dx0, ego_speed, npc_speed, str, minttc, speed_at_collide = process_a_file(args.path)
        if unit == "km":
            ego_speed, npc_speed = ego_speed*3.6, npc_speed*3.6
            speed_at_collide = speed_at_collide*3.6

        print("NPC speed, Ego speed, dx0, Is collision, Min TTC, Speed at Collide")
        print(f'{fn}, {npc_speed:.1f}, {ego_speed:.1f}, {dx0}, {str}, {minttc}, {speed_at_collide}')

    elif os.path.isdir(args.path):
        dir_path = args.path
        re = process_a_dir(dir_path)
        print("File name, NPC speed, Ego speed, dx0, Is collision, Min TTC, Speed at Collide")
        for fn, dx0, ego_speed, npc_speed, str, minttc, speed_at_collide in re:
            if unit == "km":
                ego_speed, npc_speed = ego_speed*3.6, npc_speed*3.6
                speed_at_collide = speed_at_collide*3.6
            print(f'{fn}, {npc_speed:.1f}, {ego_speed:.1f}, {dx0}, {str}, {minttc:.2f}, {speed_at_collide}')