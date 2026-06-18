from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from three_u_repro.config import PaperConfig
from three_u_repro.experiments import run_greedy_smoke, run_height_speed_sweep, run_paper_table, write_csv
from three_u_repro.svg_plots import render_standard_plots


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reproduce the 3U UAV-USV-UUV target hunting simulations.")
    parser.add_argument("--output-dir", default=str(ROOT / "outputs" / "3u_reproduction"))
    parser.add_argument("--episodes", type=int, default=300, help="DQN training episodes per case; paper scale is 10000.")
    parser.add_argument("--eval-episodes", type=int, default=30)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--only", choices=["smoke", "table", "sweep", "all"], default="smoke")
    parser.add_argument("--uav-height", type=float, default=100.0)
    parser.add_argument("--uuv-speed", type=float, default=7.8)
    parser.add_argument("--target-distance", type=float, default=100.0)
    parser.add_argument("--aco-populations", type=int, default=30, help="Use 100 for paper-scale ACO.")
    parser.add_argument("--aco-iterations", type=int, default=30, help="Use 100 for paper-scale ACO.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir)
    config = replace(
        PaperConfig(random_seed=args.seed),
        uav_height_m=args.uav_height,
        uuv_speed_kn=args.uuv_speed,
        target_initial_distance_m=args.target_distance,
        aco_populations=args.aco_populations,
        aco_iterations=args.aco_iterations,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    config.save_json(output_dir / "config.json")

    if args.only == "smoke":
        metrics = run_greedy_smoke(config)
        print("Smoke metrics:", metrics)
        print(f"Wrote config to {output_dir / 'config.json'}")
        return

    rows = []
    if args.only in {"table", "all"}:
        rows.extend(run_paper_table(config, episodes=args.episodes, eval_episodes=args.eval_episodes, seed=args.seed))
    if args.only in {"sweep", "all"}:
        rows.extend(run_height_speed_sweep(config, episodes=args.episodes, eval_episodes=args.eval_episodes, seed=args.seed))

    csv_path = output_dir / "metrics.csv"
    write_csv(csv_path, rows)
    plot_paths = render_standard_plots(csv_path, output_dir)
    print(f"Wrote metrics to {csv_path}")
    for path in plot_paths:
        print(f"Wrote plot to {path}")


if __name__ == "__main__":
    main()
