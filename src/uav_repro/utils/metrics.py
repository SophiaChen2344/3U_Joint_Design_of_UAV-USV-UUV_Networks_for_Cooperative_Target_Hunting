from __future__ import annotations

import numpy as np


def disturbance_estimation_error(true_disturbance: np.ndarray, estimated_disturbance: np.ndarray) -> np.ndarray:
    """Metric related to Eq. (35)-(40)."""
    return true_disturbance - estimated_disturbance


def formation_error_norm(errors: np.ndarray) -> np.ndarray:
    """Norm used for convergence tracking."""
    return np.linalg.norm(errors, axis=1)
