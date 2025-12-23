## Experiments with Learning-based AD Agents in CARLA

Each folder [u-turn/runX](u-turn/run1/) (where `X` is from 1 to 3) contains:
- `uturn_<agentid>_<lane>_<vo>.log`: recorded log file by using CARLA API during the U-turn scenario simulation with the specified agent, lane, and NPC vehicle speed (`vo`). These log files are replayable in the CARLA simulator. The agent IDs are as follows:
  - `if_if`: InterFuser
  - `lav_lav`: Learning from All Vehicles
  - `tf_tf`: TransFuser
  - `tl_ltf`: Latent TransFuser
  - `tf_gf`: Geometric Fusion
  - `tf_lf`: Late Fusion
- `uturn_<agentid>_<lane>_<vo>.json`: trace file containing dynamic states of the ego vehicle and other vehicles during the simulation.

Similarly, each folder [swerve/runX](swerve/run1/) (where `X` is from 1 to 3) contains the log files and trace files recorded during the swerve scenario simulations.
A file `swerve_<agentid>_<vo>_<vy>.log` or `swerve_<agentid>_<vo>_<vy>.json` corresponds to the simulation with the specified agent, NPC speed (`vo`), and lateral velocity (`vy`).


### Replaying Recorded Logs in CARLA
To replay the recorded log files in CARLA simulator, please follow these steps:

1. Install CARLA simulator (version 0.9.15 is recommended) by following the instructions in the [CARLA documentation](https://carla.readthedocs.io/en/0.9.15/start_quickstart/). Make sure to also install the client library.

2. Launch CARLA server:
   ```bash
   ./CarlaUE4.sh -vulkan
   ```

3. Use our provided Python script to replay a recorded log file:
   ```bash
   $ python scripts/replay.py <absolute-path-to-log-file>
   ```
   Make sure carla API was installed (by `pip install carla`).
   Also, make sure to use the absolute path to the log file. A relative path will not work since CARLA will will look for the file from the server side.
   For example, if you cloned the repository to your home directory, then:
    ```bash
    $ python scripts/replay.py ~/ADS-Safety-Reference-Benchmark/CARLA-agents-results/u-turn/run1/uturn_if_if_adjacent_10.log
    ```

    More options for the replay script can be viewed by:
    ```bash
    $ python scripts/replay.py -h
    usage: replay.py [-h] [-f TIME_FACTOR] [-s START] [-d DURATION] input

    CARLA Replay Viewer

    positional arguments:
    input                 Input replay file (e.g., input.rec)

    options:
    -h, --help            show this help message and exit
    -f TIME_FACTOR, --time-factor TIME_FACTOR
                            Time factor for replay speed
    -s START, --start START
                            Start time for replay
    -d DURATION, --duration DURATION
                            Duration for replay
    ```

### Reproducing the Experiments
To reproduce the experiments with the six learning-based AD agents in CARLA simulator, please follow these steps:
1. Install CARLA simulator (version 0.9.15 is recommended) by following the instructions in the [CARLA documentation](https://carla.readthedocs.io/en/0.9.15/start_quickstart/). Make sure to also install the client library.

2. Clone and install PCLA framework, which provides an unified interface to run different learning-based AD agents in CARLA simulator:
   ```bash
   $ git clone https://github.com/MasoudJTehrani/PCLA.git
   ```
   Then, follow the instructions in its [README](https://github.com/MasoudJTehrani/PCLA) to install the required dependencies.

3. Copy our Python scripts and XML route files for running the experiments from the `scripts` folder in this repository to the root folder in the cloned PCLA repository.

4. Launch CARLA server:
   ```bash
   ./CarlaUE4.sh -vulkan
   ```

5. From the PCLA folder, run the provided scripts to execute the U-turn and swerve scenarios with different agents. For example, to run the U-turn scenario with InterFuser agent:
   ```bash
   $ python uturn.py --agent if_if
   ```
   Use `--help` option to view all available options for the scripts.