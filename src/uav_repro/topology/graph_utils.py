from __future__ import annotations

import numpy as np


def in_degree(adjacency: np.ndarray) -> np.ndarray:
    """Compute the in-degree vector used in the paper."""
    return adjacency.sum(axis=1)


def laplacian(adjacency: np.ndarray) -> np.ndarray:
    """Compute L = D - A."""
    degree = np.diag(in_degree(adjacency))
    return degree - adjacency


def formation_error(
    i: int,
    positions: np.ndarray,
    desired_offsets: np.ndarray,
    leader_position: np.ndarray,
    adjacency: np.ndarray,
    pinning: np.ndarray,
) -> np.ndarray:
    """Formation error for follower i from Eq. (12) or Eq. (32)."""
    error = np.zeros(2, dtype=float)
    for j in range(len(positions)):
        error += adjacency[i, j] * ((positions[i] - desired_offsets[i]) - (positions[j] - desired_offsets[j]))
    error += pinning[i] * (positions[i] - desired_offsets[i] - leader_position)
    return error
