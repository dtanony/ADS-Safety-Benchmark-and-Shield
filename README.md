## Benchmark-Driven Safety Evaluation and Shielding of Autonomous Driving Systems

### Repository Overview
This repository contains the code and data for the paper "Benchmark-Driven Safety Evaluation and Shielding of Autonomous Driving Systems."
This repository mainly includes:

1. Safety reference benchmarks to evaluate ADSs for oncoming traffic scenarios. Check folder `safety-benchmarks` for more details. Code to reproduce the benchmarks and animations visualizing the movements of vehicles for each concrete scenario are provided in the folder.

2. Experiment results of evaluating [Autoware version 0.41.2](https://github.com/dtanony/autoware0412) (released February 20, 2025) against our safety reference benchmarks. Check folder `baseline-results` for more details.
Trace data, camera videos, and input scripts specifying all scenarios are provided in the folder.

3. Experiement results with Autoware when integrating our safety shield. Check folder `shielding-results` for more details.

\
<img src="fig-tool-chain.png" alt="Tool chain" width="500"/>

### Tools Used
Other tools used in this work are available in separate repositories:

- Extended Autoware is available [here](https://github.com/dtanony/autoware0412). This extended version supports activating AEB on demand by sending service requests to it.

- Extended AWSIM-Labs simulator is available [here](https://github.com/dtanony/awsim-labs). This extended version supports simulating U-turn and swerve behaviors of vehicles, and allows AWSIM-Script clients to  issue simulation actions for traffic participants dynamically.

- AWSIM-Script client library is available [here](https://github.com/dtanony/awsim-script). This library provides Python APIs to interact with the AWSIM-Labs simulator.

- AW-RuntimeMonitor is available [here](https://github.com/dtanony/aw-runtime-monitor). This monitor essentially consists of two subcomponents:
  - A trace recorder that logs the state (position, velocity, etc.) of the ego vehicle and other objects during simulation, camera videos, ADS internal states (e.g., perceived objects and control commands), etc.
  - A shield for the control module that checks the safety of issued control commands. If a command is unsafe, the shield activates AEB.

### Experiement with Autoware
To reproduce the experiment results with Autoware, please follow the following steps.

#### 1. Clone all required repositories
```bash
cd ~
git clone https://github.com/dtanony/ADS-Safety-Benchmark-and-Shield.git
git clone https://github.com/dtanony/autoware0412.git autoware
git clone https://github.com/dtanony/awsim-script.git
git clone https://github.com/dtanony/aw-runtime-monitor.git
```

#### 2. Autoware installation
To run Autoware end-to-end simulation with AWSIM-Labs simulator, PC with GPU is required. 

#### 3. AWSIM-Labs
Download binary file from here:

#### 4. Launch Autoware

```bash
cd ~/autoware
ros2 launch ...
```

#### 5. Launch AWSIM-Labs simulator
In another terminal, run:
```bash
./awsim-labs -noise false
```
Note that the `-noise false` option disables noise from LiDAR sensors in the simulator. By default, noise is enabled.

#### 6. Launch AW-RuntimeMonitor
In another terminal, run:
```bash
cd ~/aw-runtime-monitor
source ~/autoware/install/setup.bash
python main.py ... -v false
```
Note that for the first time, you need to install required Python packages by running `pip install -r requirements.txt`. It is recommended to use a virtual environment. Note also that you need to source Autoware's setup file before launching the monitor.
`python main.py -h` provides more details about the tool usage and the available options.

#### 7. Launch AWSIM-Script client:
In another terminal, run:
```bash
cd ~/awsim-script
source ~/autoware/install/setup.bash
python client.py ~/ADS-Safety-Benchmark-and-Shield/safety-benchmarks/Scripts/Uturn/
```

Simularly, you need to install required Python packages by running `pip install -r requirements.txt` for the first time and to source Autoware's setup file before launching the client.
`python client.py -h` provides more details about the tool usage and the available options.
