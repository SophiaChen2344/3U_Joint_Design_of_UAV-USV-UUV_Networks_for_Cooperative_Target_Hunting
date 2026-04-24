from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class FixedTimeControllerConfig:
    alpha_ix: float
    alpha_iy: float
    beta_ix: float
    beta_iy: float
    p_i: float
    m_i: int
    n_i: int
    a1: float
    b1: float
    c1: int
    k1: float


class FixedTimeController:
    """Skeleton implementation for Eq. (14)-(16) and Eq. (34)."""

    def __init__(self, cfg: FixedTimeControllerConfig) -> None:
        self.cfg = cfg

    def phi(self, e_i_rho: np.ndarray) -> float:
        norm_term = np.linalg.norm(e_i_rho.T @ e_i_rho / 2.0)
        return self.cfg.a1 + (1.0 - self.cfg.a1) * np.exp(-self.cfg.b1 * norm_term ** self.cfg.c1)

    def q_i(self, e_i_rho: np.ndarray) -> float:
        ratio = self.cfg.m_i / self.cfg.n_i
        sign_term = np.sign((e_i_rho.T @ e_i_rho) / 2.0 - 1.0)
        return ratio + (ratio - 1.0) * sign_term

    def compute_virtual_control(self, *_args, **_kwargs) -> np.ndarray:
        """Return Eq. (14) or Eq. (34) once all terms are wired in."""
        raise NotImplementedError("Wire Eq. (14)/(34) terms into this method.")

    def to_speed_and_heading(self, f_u_i: np.ndarray) -> tuple[float, float]:
        """Map virtual control to Eq. (15)."""
        v_i = float(np.linalg.norm(f_u_i))
        theta_id = float(np.arctan2(f_u_i[1], f_u_i[0]))
        return v_i, theta_id

    def angular_velocity(self, theta_i: float, theta_id: float, theta_id_dot: float) -> float:
        """Angular velocity control from Eq. (16)."""
        e_i_theta = theta_i - theta_id
        return -self.cfg.k1 * e_i_theta + theta_id_dot
