## Autoware Baseline Experiments

This folder contains:
- Input script files (to be fed into AWSIM-Script) for simulating U-turn and swerve scenarios in Autoware (both original and shielded versions) and AWSIM-Labs environment (folder [u-turn/scripts](u-turn/scripts/) and [swerve/scripts](swerve/scripts/)).
- Trace files recorded during the experiments with the original Autoware, including videos and JSON logs (folder [u-turn/data](u-turn/data/) and [swerve/data](swerve/data/)).

### U-Turn Scenarios
The U-turn experiments consist of two configurations:

1. when the ego vehicle traveling in the innermost lane, i.e., rightmost
lane under left-hand traffic and leftmost lane otherwise (folder `innermost-lane`)

2. when the ego vehicle traveling in the lane adjacent to the innermost lane (folder `adjacent-lane`).

For each configuration, the mapping between input script files and trace files is shown in the following table.

| **Script file** | **Trace file** | **Ego Speed** | **NPC Speed** |
|-----------------|----------------|---------------|---------------|
| uturn-14-10     | uturn_sim1     | 14            | 10            |
| uturn-14-15     | uturn_sim2     | 14            | 15            |
| uturn-20-10     | uturn_sim3     | 20            | 10            |
| uturn-20-15     | uturn_sim4     | 20            | 15            |
| uturn-25-10     | uturn_sim5     | 25            | 10            |
| uturn-25-15     | uturn_sim6     | 25            | 15            |
| uturn-30-10     | uturn_sim7     | 30            | 10            |
| uturn-30-15     | uturn_sim8     | 30            | 15            |
| uturn-35-10     | uturn_sim9     | 35            | 10            |
| uturn-35-15     | uturn_sim10    | 35            | 15            |
| uturn-40-10     | uturn_sim11    | 40            | 10            |
| uturn-40-15     | uturn_sim12    | 40            | 15            |


### Swerve Scenarios
The swerve experiments, the oncoming vehicle's speed (NPC speed) was set to 10 (folder `vo-10`) and 15 (folder `vo-15`).
For each NPC speed `{X}`, the mapping between input script files and trace files is shown in the following table.

| **Script file**  | **Trace file** | **Ego Speed** | **Lateral velocity** |
|------------------|----------------|---------------|----------------------|
| swerve-14-{X}-10 | swerve_sim1    | 14            | 1.0                  |
| swerve-14-{X}-12 | swerve_sim2    | 14            | 1.2                  |
| swerve-14-{X}-14 | swerve_sim3    | 14            | 1.4                  |
| swerve-20-{X}-10 | swerve_sim4    | 20            | 1.0                  |
| swerve-20-{X}-12 | swerve_sim5    | 20            | 1.2                  |
| swerve-20-{X}-14 | swerve_sim6    | 20            | 1.4                  |
| swerve-30-{X}-10 | swerve_sim7    | 30            | 1.0                  |
| swerve-30-{X}-12 | swerve_sim8    | 30            | 1.2                  |
| swerve-30-{X}-14 | swerve_sim9    | 30            | 1.4                  |
| swerve-40-{X}-10 | swerve_sim10   | 40            | 1.0                  |
| swerve-40-{X}-12 | swerve_sim11   | 40            | 1.2                  |
| swerve-40-{X}-14 | swerve_sim12   | 40            | 1.4                  |


### Experiment Reproduction
To run the experiments with Autoware, the following tools, which are available in separate repositories, are required:

- Extended [Autoware](https://github.com/dtanony/autoware0412): This extended version supports activating AEB on demand by sending ROS 2 service requests to it.

- Extended [AWSIM-Labs simulator](https://github.com/dtanony/AWSIM-Labs): This extended version supports simulating U-turn and swerve behaviors of vehicles, and allows AWSIM-Script clients to  issue simulation actions for traffic participants dynamically.

- Extended [AWSIM-Script](https://github.com/dtanony/AWSIM-Script-Client): This library provides Python APIs to interact with the AWSIM-Labs simulator.

- [AW-Runtime-Monitor](https://github.com/dtanony/AW-Runtime-Monitor): This monitor essentially consists of two subcomponents:
  - A trace recorder that logs the state (position, velocity, etc.) of the ego vehicle and other objects during simulation, camera videos, ADS internal states (e.g., perceived objects and control commands), etc.
  - A shield for the control module that checks the safety of issued control commands. If a command is unsafe, the shield activates AEB.

Please follow the following steps.

#### 1. Launch AWSIM-Labs
Instructions to launch AWSIM-Labs are provided in its [repository](https://github.com/dtanony/AWSIM-Labs).
First, complete the [Prerequisite Setup](https://github.com/dtanony/AWSIM-Labs?tab=readme-ov-file#prerequisite-setup)
and [Driver Installation](https://github.com/dtanony/AWSIM-Labs?tab=readme-ov-file#driver-installation-skip-if-already-installed).

To launch AWSIM-Labs,
we recommend to use the binary release, which can be downloaded from
[here](https://github.com/dtanony/AWSIM-Labs/releases/download/v1.0/awsim_labs.zip).
Unzip it and launch the simulator using:

```bash
./awsim_labs.x86_64 -noise false
```

Note that the option `-noise false` disables Gaussian noise in the simulated data from LiDAR sensors.
By default, noise is enabled.

#### 2. Launch Autoware
Instructions to install and launch Autoware are provided in its [repository](https://github.com/dtanony/autoware0412).
To run an end-to-end Autoware simulation with the AWSIM-Labs simulator, a PC equipped with a GPU is required. 
Because of the specific GPU driver and CUDA dependencies, a pre-built binary release of Autoware is not available for this setup. 
Therefore, the only option is to build Autoware from source.
Follow the provided steps there to install it.
Once succeeded, launch Autoware with the following commands in another terminal:

```bash
cd ~/autoware  # Assume Autoware is installed in the home directory
source install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=<your-map-folder>/nishishinjuku_autoware_map launch_vehicle_interface:=true
```

Note that you need to use the absolute path for the map folder, don't use the ~ operator.

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/your_username/autoware_map/nishishinjuku_autoware_map launch_vehicle_interface:=true
```

#### 3. Launch AW-Runtime-Monitor
Instructions to install and launch AW-Runtime-Monitor are available in its [repository](https://github.com/dtanony/AW-Runtime-Monitor).

After launching Autoware and AWSIM-Labs and they are connected, run the following command in another terminal:
```bash
python main.py -o <path-to-folder-to-save-traces> -v false
```

where the options `-v false` disable shielding. By default, it is enabled.
Note that you need to source Autoware's setup file before launching the monitor.
For more details about the tool usage, use `python main.py -h`.

```bash
$ python main.py -h
usage: main.py [-h] [-o OUTPUT] [-f {json,yaml}] [-n NO_SIM] [-v {true,false}]

Runtime Monitor for Autoware and AWSIM simulator. Adjust the component to record data by modifying
file config.yaml

options:
  -h, --help            show this help message and exit
  -o OUTPUT, --output OUTPUT
                        Output trace file name (default: auto-generated with timestamp)
  -f {json,yaml}, --format {json,yaml}
                        either json or yaml (default: json)
  -n NO_SIM, --no_sim NO_SIM
                        Simulation number, use as suffix to the file name (default: 1)
  -v {true,false}, --verify_control_cmd {true,false}
                        To verify the safety of control commands, i.e., enable shielding (true or
                        false, default: true)
```

#### 4. Launch AWSIM-Script client:
Instructions to install and launch AWSIM-Script-Client are available in its [repository](https://github.com/dtanony/AWSIM-Script-Client).

For example, to execute U-turn scenarios when the ego vehicle travels on the adjacent lane to the rightmost lane, run the following command in another terminal:
```bash
python client.py Autoware-baseline-results/u-turn/scripts/adjacent-lane
```
Each scenario in the folder will be executed sequentially. When a scenario terminates (i.e., when the ego vehicle reaches its goal), the recorded data will be saved to folder `<path-to-folder-to-save-traces>` (provided when running AW-Runtime-Monitor) with incremental numbering.
