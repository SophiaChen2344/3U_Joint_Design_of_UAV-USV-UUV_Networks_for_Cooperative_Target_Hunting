# Fixed-Time Networked UAV Topology Reconfiguration Reproduction

This repository scaffolds a Python reproduction of the IEEE paper:

`Fixed-Time Networked UAV Topology Reconfiguration With Disturbance Rejection via Deep Reinforcement Learning`

Current implementation direction:

- MATLAB-first for all subsequent work
- Fixed-topology formation control only
- USDE disturbance estimation enabled
- DDPG topology reconfiguration deferred

The project is organized around the paper's mathematical modules:

- UAV kinematics: Eq. (6)-(7)
- USDE disturbance estimator: Eq. (8)-(11)
- Formation error and graph model: Eq. (12)-(13), Eq. (32)-(33)
- Fixed-time controller: Eq. (14)-(16), Eq. (34)
- DDPG topology reconfiguration: Eq. (17)-(31)
- Stability checks: Eq. (35)-(43)

## MATLAB Quick Start

```matlab
cd('C:\Users\lenovo\Documents\New project\matlab')
run_fixed_topology_main
```

## Structure

```text
matlab/      MATLAB entry script and modules for fixed-topology control
configs/     archived parameter templates from earlier scaffold
docs/        formula mappings and experiment notes
scripts/     runnable entry points
src/         source code modules
outputs/     logs, figures, checkpoints
```

## Current Scope

1. Implement and validate UAV dynamics and USDE in MATLAB.
2. Implement fixed-time controller under a fixed topology.
3. Reproduce the fixed-topology part of the paper before any topology switching.
4. Add baselines only after the nominal fixed-topology controller is stable.

## Notes

- The Python scaffold remains in the repository, but it is no longer the active path.
- The active workstream is under `matlab/`.
