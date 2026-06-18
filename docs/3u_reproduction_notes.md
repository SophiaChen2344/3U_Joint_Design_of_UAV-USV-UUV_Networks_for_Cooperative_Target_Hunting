# 3U Paper Reproduction Notes

This project reproduces the simulation workflow for **“3U: Joint Design of UAV-USV-UUV Networks for Cooperative Target Hunting”** with a runnable Python implementation.

## Implemented Scope

- 3U geometry: one UAV, one USV relay, and a UUV cluster center in a `400 x 400 m` region.
- UUV pursuit actions: eight discrete moving directions, matching the DQN action definition.
- Target escape rule: the target moves away from the UUV cluster center at `Vt`.
- Objective metrics: UUV motion energy, voyage distance, UAV-USV distance, USV-UUV distance, and success rate.
- Algorithms: NumPy DQN, Double DQN, Dueling DQN, and an ACO-style baseline.
- Outputs: CSV metrics and SVG plots corresponding to Fig. 2, Fig. 3, and Table II style comparisons.

## Equation Map

- Eq. (2)-(3): UAV-USV connectivity is implemented as a distance-decay probability surrogate.
- Eq. (4): USV-UUV connectivity is implemented from the eigenvalues of a weighted vehicle graph.
- Eq. (5): acoustic absorption is documented but not used in the main evaluation because the paper also evaluates `EUUV` with motion energy only.
- Eq. (6): constraints are represented by altitude, search-region, and underwater-connectivity checks.
- Eq. (7): reward follows terminal, positive-progress, and negative-constraint/failure cases.
- Eq. (8): DQN target update is implemented with optional Double DQN target selection.

## Paper Parameters

The defaults in `configs/3u_default.json` follow Table I:

- UAV: `(200, 200, h)`, `h in [50, 120] m`
- USV: `(200, 200, 0)`, `VS = 3.9 kn`
- UUV center: `(200, 200, -120)`, `M = 3`, `VG = 3.9-27.3 kn`
- Target: `Vt = 1 kn`, initial distance `H = 100 m`
- Energy: `epsilon = 80%`, `Fd = 2000 N`
- DQN: two hidden layers, batch size `128`, memory `10000`, discount `0.95`, rewards `(10, 0.1, -1)`
- ACO: population `100`, iterations `100`, pheromone volatility `0.2`

## Reproduction Assumptions

The paper is a short IEEE letter and does not fully specify several constants needed for exact byte-for-byte numerical reproduction. This implementation keeps those choices explicit:

- UAV coverage radius uses `r = 2.04 h`; this keeps `h=100 m` on the same scale as Table II voyage distances.
- The USV follows a relay point between UAV and UUV center because the paper says the USV acts as a relay but does not define a DQN action for USV control.
- The reward terminal condition uses the safe/capture radius `r2 = 15 m`; the text mixes `r` and `r2` in the target-hunting description.
- Communication energy is omitted from default metrics, matching the paper’s evaluation statement that motion energy dominates.

## Commands

Quick smoke test:

```powershell
python scripts\run_3u_reproduction.py --only smoke
```

Small reproducible run:

```powershell
python scripts\run_3u_reproduction.py --only all --episodes 300 --eval-episodes 30 --aco-populations 30 --aco-iterations 30
```

Paper-scale run:

```powershell
python scripts\run_3u_reproduction.py --only all --episodes 10000 --eval-episodes 100 --aco-populations 100 --aco-iterations 100
```
