from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class StepResult:
    state: np.ndarray
    reward: float
    done: bool
    info: dict


class FormationEnv:
    """Environment wrapper that will couple dynamics, observer, controller, and topology."""

    def __init__(self, config: dict) -> None:
        self.config = config

    def reset(self) -> np.ndarray:
        raise NotImplementedError("Initialize UAV states, topology, and disturbances.")

    def step(self, action: float | None = None) -> StepResult:
        raise NotImplementedError("Advance one simulation step and return Eq. (17)-(23) quantities.")
