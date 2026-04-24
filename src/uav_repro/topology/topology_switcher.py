from __future__ import annotations


def action_to_topology_index(action: float, num_topologies: int) -> int:
    """Map a continuous DDPG action to a discrete topology index using Eq. (18)."""
    bounded = min(max(float(action), 0.0), 1.0)
    for idx in range(1, num_topologies + 2):
        lower = (idx - 1) / (num_topologies + 1)
        upper = idx / (num_topologies + 1)
        if lower < bounded <= upper:
            return idx - 1
    return 0
