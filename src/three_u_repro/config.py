from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import json
from typing import Any


KNOT_TO_MPS = 0.514444


@dataclass
class PaperConfig:
    """Default parameters from Table I and the simulation paragraph.

    A few radio/acoustic constants are not fully specified in the paper, so
    they are exposed here and documented in docs/3u_reproduction_notes.md.
    """

    area_size: float = 400.0
    num_uuv: int = 3
    uav_xy: tuple[float, float] = (200.0, 200.0)
    usv_initial_xy: tuple[float, float] = (200.0, 200.0)
    uuv_initial_xy: tuple[float, float] = (200.0, 200.0)
    uav_height_m: float = 100.0
    h_min_m: float = 50.0
    h_max_m: float = 120.0
    underwater_depth_m: float = -120.0
    safe_radius_m: float = 15.0
    target_initial_distance_m: float = 100.0
    usv_speed_kn: float = 3.9
    uuv_speed_kn: float = 7.8
    target_speed_kn: float = 1.0
    conversion_efficiency: float = 0.80
    drag_force_n: float = 2000.0
    connectivity_c1: float = 0.00007
    energy_balance_c2: float = 40.0

    dqn_iterations: int = 10_000
    dqn_learning_rate: float = 0.001
    dqn_discount: float = 0.95
    dqn_batch_size: int = 128
    dqn_memory_capacity: int = 10_000
    dqn_paper_epsilon: float = 0.9
    terminal_reward: float = 10.0
    positive_reward: float = 0.1
    negative_reward: float = -1.0

    aco_populations: int = 100
    aco_iterations: int = 100
    aco_pheromone_volatility: float = 0.2

    time_step_s: float = 1.0
    max_steps: int = 220
    random_seed: int = 7

    # Practical surrogate constants for under-specified equations.
    search_radius_per_altitude: float = 2.04
    usv_relay_fraction_to_uuv: float = 0.55
    acoustic_link_radius_m: float = 260.0
    em_link_scale_m: float = 260.0
    underwater_link_scale_m: float = 180.0
    uav_transmit_power: float = 260.0
    rayleigh_mean: float = 1.0
    sinr_threshold: float = 1.0
    noise_power: float = 1.0
    expected_interference: float = 0.0
    path_loss_exponent: float = 1.0
    uav_usv_pc_min: float = 0.01

    @property
    def usv_speed_mps(self) -> float:
        return self.usv_speed_kn * KNOT_TO_MPS

    @property
    def uuv_speed_mps(self) -> float:
        return self.uuv_speed_kn * KNOT_TO_MPS

    @property
    def target_speed_mps(self) -> float:
        return self.target_speed_kn * KNOT_TO_MPS

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_json(cls, path: str | Path) -> "PaperConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(**data)

    def save_json(self, path: str | Path) -> None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
