## Benchmark-Driven Safety Evaluation and Shielding of Autonomous Driving Systems

### Repository Overview
This repository contains the code and data for the paper "Benchmark-Driven Safety Evaluation and Shielding of Autonomous Driving Systems."
This repository mainly includes:

1. Safety reference benchmarks to evaluate ADSs for oncoming traffic scenarios. Check folder `safety-benchmarks` for more details. Code to reproduce the benchmarks and animations visualizing the movements of vehicles for each concrete scenario are provided in the folder.

2. Experiment results of evaluating [Autoware version 0.41.2](https://github.com/dtanony/Autoware0412) (released February 20, 2025) against our safety reference benchmarks. Check folder `baseline-results` for more details.
Trace data, camera videos, and input scripts specifying all scenarios are provided in the folder.

3. Experiement results with Autoware when integrating our safety shield. Check folder `shielding-results` for more details.

\
<img src="fig-tool-chain.png" alt="Tool chain" width="500"/>

### Tools Used
Other tools used in this work are available in separate repositories:

- Extended Autoware is available [here](https://github.com/dtanony/autoware0412). This extended version supports activating AEB on demand by sending service requests to it.

- Extended AWSIM-Labs simulator is available [here](https://github.com/dtanony/AWSIM-Labs). This extended version supports simulating U-turn and swerve behaviors of vehicles, and allows AWSIM-Script clients to  issue simulation actions for traffic participants dynamically.

- AWSIM-Script client library is available [here](https://github.com/dtanony/AWSIM-Script-Client). This library provides Python APIs to interact with the AWSIM-Labs simulator.

- AW-RuntimeMonitor is available [here](https://github.com/dtanony/AW-Runtime-Monitor). This monitor essentially consists of two subcomponents:
  - A trace recorder that logs the state (position, velocity, etc.) of the ego vehicle and other objects during simulation, camera videos, ADS internal states (e.g., perceived objects and control commands), etc.
  - A shield for the control module that checks the safety of issued control commands. If a command is unsafe, the shield activates AEB.

### Experiement with Autoware
To reproduce the experiment results with Autoware, please follow the following steps.

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
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=<your-map-folder>/nishishinjuku_autoware_map launch_vehicle_interface:=true
```

#### 3. Launch AW-Runtime-Monitor
Instructions to install and launch AW-Runtime-Monitor are available in its [repository](https://github.com/dtanony/AW-Runtime-Monitor).

After launching Autoware and AWSIM-Labs and they are connected, run the following command in another terminal:
```bash
python main.py -o <path-to-folder-to-save-traces> -v false
```

where the options `-v false` disable shielding. By default, it is enabled.

#### 4. Launch AWSIM-Script client:
Instructions to install and launch AWSIM-Script-Client are available in its [repository](https://github.com/dtanony/WSIM-Script-Client).

For example, to execute U-turn scenarios, run the following command in another terminal:
```bash
python client.py ADS-Safety-Benchmark-and-Shield/safety-benchmarks/Scripts/Uturn/
```
Each scenario in the folder will be executed sequentially. When a scenario terminates (i.e., when the ego vehicle reaches its goal), the recorded data will be saved to <path-to-folder-to-save-traces> with incremental numbering.