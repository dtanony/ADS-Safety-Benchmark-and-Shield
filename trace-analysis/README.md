## Trace Analysis

This folder provides Python scripts for analyzing recorded traces.

### Usage
The example below shows the analysis results for experiments with the baseline Autoware, when the ego vehicle travels in the lane adjacent to the rightmost lane:.
```bash
$ python analysis.py ../Autoware-baseline-results/u-turn/data/adjacent-lane/run1/ -u km

...

File name, NPC speed, Ego speed, dx0, Is collision, Min TTC, Speed at Collide
uturn_sim1.json, 10.0, 20.0, 16.98, N, 0.44, 0.0
uturn_sim10.json, 15.0, 40.0, 31.196, N, 1.14, 0.0
uturn_sim2.json, 15.0, 20.0, 15.137, N, 0.77, 0.0
uturn_sim3.json, 10.0, 25.1, 21.023, N, 0.20, 0.0
uturn_sim4.json, 15.0, 25.1, 19.237, N, 0.94, 0.0
uturn_sim5.json, 10.0, 30.0, 26.173, N, 0.50, 0.0
uturn_sim6.json, 15.0, 30.0, 23.19, N, 1.04, 0.0
uturn_sim7.json, 10.0, 35.0, 30.436, Y (450.19), 0.00, 20.194194785630845
uturn_sim8.json, 15.0, 34.9, 27.015, N, 1.12, 0.0
uturn_sim9.json, 10.0, 40.0, 36.069, Y (590.243), 0.00, 24.352966009092196
```

Output information includes:
- NPC speed at the start of the U-turn/swerve. 
- Ego speed at the start of the U-turn/swerve. 
Note that the Ego speed is controlled by Autoware, not directly by us. We can only specify the maximum desired speed. The results confirm that the actual ego speeds match the desired values with negligible error.
- Longitudinal distance between the two vehicles at the start of the U-turn/swerve($dx_0$). This parameter is also not directly controlled, which explains the small discrepancies between the actual and desired values.
- Collision status (whether a collision occurred).
- Minimum TTC (Time-to-Collision) between the two vehicles (0 if a collision occurred).
- Ego speed at collision (0 if no collision).