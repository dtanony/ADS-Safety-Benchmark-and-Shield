### U-Turn Scenarios
The U-turn experiments consist of two cases: 
- when the autonomous vehicle (AV) traveled in
the rightmost lane under left-hand traffic (folder `rightmost-lane`), and
- when the AV traveled in the
lane adjacent to the rightmost lane (folder `adjacent-lane`).

For each case, the mapping between input script files and trace files is shown in the following table.

| **Script file** | **Trace file** | **AV Speed** | **NPC Speed** |
|-----------------|----------------|--------------|---------------|
| uturn-20-10-1   | uturn_sim1     | 20           | 10            |
| uturn-20-15-1   | uturn_sim2     | 20           | 15            |
| uturn-25-10-1   | uturn_sim3     | 25           | 10            |
| uturn-25-15-1   | uturn_sim4     | 25           | 15            |
| uturn-30-10-1   | uturn_sim5     | 30           | 10            |
| uturn-30-15-1   | uturn_sim6     | 30           | 15            |
| uturn-35-10-1   | uturn_sim7     | 35           | 10            |
| uturn-35-15-1   | uturn_sim8     | 35           | 15            |
| uturn-40-10-1   | uturn_sim9     | 40           | 10            |
| uturn-40-15-1   | uturn_sim10    | 40           | 15            |


### Swerve Scenarios
The swerve experiments, the oncoming vehicle's speed (NPC speed) was set to 10 (folder `vo-10`), 15 (folder `vo-15`), and 20 km/h (folder `vo-20`).
For each NPC speed `{X}`, the mapping between input script files and trace files is shown in the following table.

| **Script file**  | **Trace file** | **AV Speed** | **Lateral velocity** |
|------------------|----------------|--------------|----------------------|
| swerve-20-{X}-10 | swerve_sim1    | 20           | 1.0                  |
| swerve-20-{X}-12 | swerve_sim2    | 20           | 1.2                  |
| swerve-20-{X}-14 | swerve_sim3    | 20           | 1.4                  |
| swerve-30-{X}-10 | swerve_sim4    | 30           | 1.0                  |
| swerve-30-{X}-12 | swerve_sim5    | 30           | 1.2                  |
| swerve-30-{X}-14 | swerve_sim6    | 30           | 1.4                  |
| swerve-40-{X}-10 | swerve_sim7    | 40           | 1.0                  |
| swerve-40-{X}-12 | swerve_sim8    | 40           | 1.2                  |
| swerve-40-{X}-14 | swerve_sim9    | 40           | 1.4                  |
