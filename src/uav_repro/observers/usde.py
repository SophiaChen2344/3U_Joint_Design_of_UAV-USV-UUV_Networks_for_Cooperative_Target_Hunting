from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class USDEState:
    p_if: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    v_if: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))


class USDE:
    """Unknown system dynamics estimator based on Eq. (8)-(11)."""

    def __init__(self, kappa: float) -> None:
        self.kappa = float(kappa)
        self.state = USDEState()

    def reset(self) -> None:
        self.state = USDEState()

    def step(self, p_i: np.ndarray, v_i: np.ndarray, dt: float) -> np.ndarray:
        """Update the low-pass filters and return disturbance estimate."""
        self.state.p_if += dt * (p_i - self.state.p_if) / self.kappa
        self.state.v_if += dt * (v_i - self.state.v_if) / self.kappa
        return -self.state.v_if + (p_i - self.state.p_if) / self.kappa
