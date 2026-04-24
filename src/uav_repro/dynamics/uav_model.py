from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class UAVState:
    x: float
    y: float
    theta: float


@dataclass
class UAVControl:
    v: float
    w: float


def follower_derivative(state: UAVState, control: UAVControl, disturbance: np.ndarray) -> np.ndarray:
    """Follower planar kinematics from Eq. (6)."""
    return np.array(
        [
            control.v * np.cos(state.theta) + disturbance[0],
            control.v * np.sin(state.theta) + disturbance[1],
            control.w,
        ],
        dtype=float,
    )


def leader_derivative(state: UAVState, control: UAVControl) -> np.ndarray:
    """Leader planar kinematics from Eq. (7)."""
    return np.array(
        [
            control.v * np.cos(state.theta),
            control.v * np.sin(state.theta),
            control.w,
        ],
        dtype=float,
    )
