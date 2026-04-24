from __future__ import annotations

import numpy as np


def dense_reward(errors: np.ndarray) -> float:
    """Dense reward term from Eq. (20)."""
    return -float(np.sum(np.abs(errors[:, 0])) + np.sum(np.abs(errors[:, 1])))


def sparse_reward(errors: np.ndarray, r1: float, r2: float, varpi1: float, varpi2: float) -> float:
    """Sparse reward terms from Eq. (21)-(22)."""
    reward = 0.0
    if np.sum(np.abs(errors[:, 0])) < varpi1:
        reward += r1
    if np.sum(np.abs(errors[:, 1])) < varpi2:
        reward += r2
    return reward


def total_reward(errors: np.ndarray, r1: float, r2: float, varpi1: float, varpi2: float) -> float:
    """Total reward from Eq. (23)."""
    return dense_reward(errors) + sparse_reward(errors, r1, r2, varpi1, varpi2)
