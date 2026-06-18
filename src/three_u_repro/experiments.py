from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import csv
import time

import numpy as np

from .agents import evaluate_agent, run_aco, summarize_results, train_dqn
from .config import PaperConfig
from .environment import ThreeUEnvironment


def write_csv(path: Path, rows: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_paper_table(
    base_config: PaperConfig,
    *,
    episodes: int,
    eval_episodes: int,
    seed: int,
) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    target_distances = [50.0, 75.0, 100.0, 125.0]
    algorithms = [
        ("DQN-lr0.001", {"learning_rate": 0.001, "double": False, "dueling": False}),
        ("DQN-lr0.01", {"learning_rate": 0.01, "double": False, "dueling": False}),
        ("Double-DQN", {"learning_rate": 0.001, "double": True, "dueling": False}),
        ("Dueling-DQN", {"learning_rate": 0.001, "double": False, "dueling": True}),
    ]
    for target_distance in target_distances:
        config = replace(base_config, target_initial_distance_m=target_distance)
        start = time.perf_counter()
        _, aco_result = run_aco(config, seed=seed)
        rows.append(
            {
                "case": "table_ii",
                "algorithm": "ACO",
                "H_m": target_distance,
                "h_m": config.uav_height_m,
                "Vg_kn": config.uuv_speed_kn,
                "training_time_s": round(time.perf_counter() - start, 4),
                **summarize_results([aco_result]),
            }
        )
        for name, kwargs in algorithms:
            start = time.perf_counter()
            agent, history = train_dqn(config, episodes=episodes, seed=seed, **kwargs)
            metrics = evaluate_agent(config, agent, eval_episodes, seed=seed + 31)
            rows.append(
                {
                    "case": "table_ii",
                    "algorithm": name,
                    "H_m": target_distance,
                    "h_m": config.uav_height_m,
                    "Vg_kn": config.uuv_speed_kn,
                    "training_time_s": round(time.perf_counter() - start, 4),
                    "train_success_rate": summarize_results(history[-max(1, min(len(history), eval_episodes)) :])["success_rate"],
                    **metrics,
                }
            )
    return rows


def run_height_speed_sweep(
    base_config: PaperConfig,
    *,
    episodes: int,
    eval_episodes: int,
    seed: int,
) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    sweeps = [
        ("fig2a_fig3a_height", "h_m", [50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0]),
        ("fig2b_fig3b_speed", "Vg_kn", [3.9, 7.8, 11.7, 15.6, 19.5, 23.4, 27.3]),
    ]
    for case, field, values in sweeps:
        for value in values:
            if field == "h_m":
                config = replace(base_config, uav_height_m=value, uuv_speed_kn=7.8)
            else:
                config = replace(base_config, uav_height_m=100.0, uuv_speed_kn=value)
            for lr in [0.001, 0.01]:
                start = time.perf_counter()
                agent, _ = train_dqn(config, episodes=episodes, learning_rate=lr, seed=seed)
                metrics = evaluate_agent(config, agent, eval_episodes, seed=seed + 17)
                rows.append(
                    {
                        "case": case,
                        "algorithm": f"DQN-lr{lr}",
                        "H_m": config.target_initial_distance_m,
                        "h_m": config.uav_height_m,
                        "Vg_kn": config.uuv_speed_kn,
                        "training_time_s": round(time.perf_counter() - start, 4),
                        **metrics,
                    }
                )
            start = time.perf_counter()
            _, aco_result = run_aco(config, seed=seed)
            rows.append(
                {
                    "case": case,
                    "algorithm": "ACO",
                    "H_m": config.target_initial_distance_m,
                    "h_m": config.uav_height_m,
                    "Vg_kn": config.uuv_speed_kn,
                    "training_time_s": round(time.perf_counter() - start, 4),
                    **summarize_results([aco_result]),
                }
            )
    return rows


def run_greedy_smoke(config: PaperConfig) -> dict[str, float]:
    env = ThreeUEnvironment(config)
    state = env.reset(target_angle_rad=0.0)
    del state
    results = []
    for _ in range(5):
        env.reset()
        done = False
        info = env.metrics()
        while not done:
            direction = env.target_xy - env.uuv_xy
            action = int(np.argmax(ACTION_DOT_CACHE @ (direction / max(np.linalg.norm(direction), 1e-12))))
            _, _, done, info = env.step(action)
        results.append(info)
    return {
        "success_rate": float(sum(info.success for info in results) / len(results)),
        "mean_energy_kj": float(np.mean([info.energy_kj for info in results])),
    }


ACTION_DOT_CACHE = np.array(
    [
        [1.0, 0.0],
        [2**-0.5, 2**-0.5],
        [0.0, 1.0],
        [-2**-0.5, 2**-0.5],
        [-1.0, 0.0],
        [-2**-0.5, -2**-0.5],
        [0.0, -1.0],
        [2**-0.5, -2**-0.5],
    ],
    dtype=np.float64,
)
