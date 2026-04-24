from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Topology:
    name: str
    adjacency: np.ndarray
    pinning: np.ndarray


def load_candidate_topologies() -> list[Topology]:
    """Return candidate graphs once Fig. 2 is digitized."""
    return []
