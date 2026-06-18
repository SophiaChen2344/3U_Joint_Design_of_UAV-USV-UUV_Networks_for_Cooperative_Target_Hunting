# 3U UAV-USV-UUV Cooperative Target Hunting Reproduction

This repository now includes a runnable reproduction scaffold for the IEEE TVT paper:

`3U: Joint Design of UAV-USV-UUV Networks for Cooperative Target Hunting`

The implementation focuses on the paper's simulation model:

- 3U system geometry: UAV, USV relay, and UUV cluster center
- Energy-oriented target hunting objective
- UAV-USV and USV-UUV connectivity metrics
- Eight-direction UUV action space
- DQN, Double DQN, Dueling DQN, and ACO-style baseline
- CSV and SVG outputs for Fig. 2, Fig. 3, and Table II style comparisons

## Quick Start

The local `python` on this machine currently has a broken NumPy install. The Codex bundled Python works:

```powershell
$py='C:\Users\lenovo\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
& $py scripts\run_3u_reproduction.py --only smoke
```

Small end-to-end run:

```powershell
& $py scripts\run_3u_reproduction.py --only all --episodes 300 --eval-episodes 30 --aco-populations 30 --aco-iterations 30
```

Paper-scale run:

```powershell
& $py scripts\run_3u_reproduction.py --only all --episodes 10000 --eval-episodes 100 --aco-populations 100 --aco-iterations 100
```

Outputs are written to:

```text
outputs/3u_reproduction/
```

## Key Files

```text
src/three_u_repro/          3U simulation, DQN, ACO, metrics, plotting
scripts/run_3u_reproduction.py
configs/3u_default.json
docs/3u_reproduction_notes.md
```

## Notes

- The previous fixed-time UAV topology scaffold is still present under `src/uav_repro/` and `matlab/`, but it is not the active path for this 3U paper.
- The paper does not specify every constant required for exact numerical replication, so reproduction assumptions are listed in `docs/3u_reproduction_notes.md`.
