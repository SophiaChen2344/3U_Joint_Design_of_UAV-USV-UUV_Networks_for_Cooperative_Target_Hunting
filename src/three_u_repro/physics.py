from __future__ import annotations

import math
import numpy as np

from .config import PaperConfig


ACTION_DIRECTIONS = np.array(
    [
        [1.0, 0.0],
        [math.sqrt(0.5), math.sqrt(0.5)],
        [0.0, 1.0],
        [-math.sqrt(0.5), math.sqrt(0.5)],
        [-1.0, 0.0],
        [-math.sqrt(0.5), -math.sqrt(0.5)],
        [0.0, -1.0],
        [math.sqrt(0.5), -math.sqrt(0.5)],
    ],
    dtype=np.float64,
)


def norm(vector: np.ndarray) -> float:
    return float(np.linalg.norm(vector))


def unit(vector: np.ndarray) -> np.ndarray:
    magnitude = norm(vector)
    if magnitude < 1e-12:
        return np.zeros_like(vector, dtype=np.float64)
    return vector / magnitude


def clamp_xy(xy: np.ndarray, area_size: float) -> np.ndarray:
    return np.clip(xy, 0.0, area_size)


def motion_energy_kj(path_length_m: float, config: PaperConfig) -> float:
    return config.conversion_efficiency * config.drag_force_n * path_length_m / 1000.0


def uav_search_radius_m(height_m: float, config: PaperConfig) -> float:
    """Coverage-radius surrogate used by the reproduction.

    The paper states Eq. (1), but the environment-dependent constants required
    to solve it are not provided in Table I. The default ratio is calibrated so
    h=100 m gives a roughly 204 m coverage radius, consistent with Table II's
    scale of optimized UUV voyage distances.
    """

    return config.search_radius_per_altitude * height_m


def uav_usv_connectivity(uav_xy: np.ndarray, usv_xy: np.ndarray, height_m: float, config: PaperConfig) -> float:
    """Probability of successful UAV-USV transmission from Eq. (3)."""

    distance = math.sqrt(norm(uav_xy - usv_xy) ** 2 + height_m**2)
    exponent = -(
        config.sinr_threshold * (distance**config.path_loss_exponent) * config.noise_power
        + config.expected_interference
    ) / max(config.rayleigh_mean * config.uav_transmit_power, 1e-9)
    return math.exp(exponent)


def underwater_connectivity(usv_xy: np.ndarray, uuv_center_xy: np.ndarray, config: PaperConfig) -> float:
    """Connectivity metric matching the eigenvalue form of Eq. (4)."""

    formation_radius = max(config.safe_radius_m * 0.45, 1.0)
    offsets = np.array(
        [
            [formation_radius, 0.0],
            [-0.5 * formation_radius, math.sqrt(3.0) * 0.5 * formation_radius],
            [-0.5 * formation_radius, -math.sqrt(3.0) * 0.5 * formation_radius],
        ],
        dtype=np.float64,
    )
    uuv_positions = uuv_center_xy + offsets[: config.num_uuv]
    positions = np.vstack([usv_xy, uuv_positions])
    count = positions.shape[0]
    psi = np.zeros((count, count), dtype=np.float64)
    for i in range(count):
        for j in range(count):
            if i == j:
                continue
            distance = norm(positions[i] - positions[j])
            if distance <= config.acoustic_link_radius_m:
                psi[i, j] = 1.0 / max(distance, 1e-9)
    eigenvalues = np.linalg.eigvals(psi).real
    return float(np.log(np.mean(np.exp(eigenvalues))))


def energy_balance_gap(config: PaperConfig) -> float:
    """Energy-balance term in Eq. (6e) for the cluster-center abstraction.

    The paper later ignores individual UUV formation details and represents
    nearby UUVs by one center G. Under this abstraction, every UUV follows the
    same center trajectory and consumes the same motion energy, so Eq. (6e)'s
    deviation from the mean is zero.
    """

    return 0.0


def relay_target_xy(uav_xy: np.ndarray, uuv_xy: np.ndarray, config: PaperConfig) -> np.ndarray:
    fraction = config.usv_relay_fraction_to_uuv
    return (1.0 - fraction) * uav_xy + fraction * uuv_xy
