from __future__ import annotations

import numpy as np


def encode_state(errors: np.ndarray, norm_x: np.ndarray, norm_y: np.ndarray) -> np.ndarray:
    """Construct the MDP state from Eq. (17)."""
    e_rho_x = errors[:, 0] / norm_x
    e_rho_y = errors[:, 1] / norm_y
    return np.concatenate([e_rho_x, e_rho_y], axis=0)
