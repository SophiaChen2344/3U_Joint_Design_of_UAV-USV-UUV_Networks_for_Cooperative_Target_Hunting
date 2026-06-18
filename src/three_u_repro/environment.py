from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from .config import PaperConfig
from .physics import (
    ACTION_DIRECTIONS,
    clamp_xy,
    energy_balance_gap,
    motion_energy_kj,
    norm,
    relay_target_xy,
    uav_search_radius_m,
    uav_usv_connectivity,
    underwater_connectivity,
    unit,
)


@dataclass
class StepInfo:
    distance_m: float
    path_length_m: float
    energy_kj: float
    uav_usv_distance_m: float
    usv_uuv_distance_m: float
    pc: float
    delta_bar: float
    energy_balance_gap: float
    success: bool
    constraint_ok: bool


class ThreeUEnvironment:
    """2-D pursuit environment for the 3U cooperative target hunting model."""

    action_size = len(ACTION_DIRECTIONS)

    def __init__(self, config: PaperConfig, seed: int | None = None):
        self.config = config
        self.rng = np.random.default_rng(config.random_seed if seed is None else seed)
        self.uav_xy = np.array(config.uav_xy, dtype=np.float64)
        self.state_size = 15
        self.reset()

    def reset(
        self,
        *,
        target_distance_m: float | None = None,
        target_angle_rad: float | None = None,
    ) -> np.ndarray:
        config = self.config
        self.usv_xy = np.array(config.usv_initial_xy, dtype=np.float64)
        self.uuv_xy = np.array(config.uuv_initial_xy, dtype=np.float64)
        self.path_length_m = 0.0
        self.steps = 0
        self.done = False
        distance = config.target_initial_distance_m if target_distance_m is None else target_distance_m
        angle = float(self.rng.uniform(0.0, 2.0 * math.pi) if target_angle_rad is None else target_angle_rad)
        self.target_xy = self.uuv_xy + distance * np.array([math.cos(angle), math.sin(angle)], dtype=np.float64)
        self.target_xy = clamp_xy(self.target_xy, config.area_size)
        self.previous_distance_m = norm(self.target_xy - self.uuv_xy)
        return self.observation()

    def observation(self) -> np.ndarray:
        config = self.config
        xy_scale = config.area_size
        z_scale = max(abs(config.underwater_depth_m), config.h_max_m, 1.0)
        escape_direction = unit(self.target_xy - self.uuv_xy)
        return np.array(
            [
                self.uav_xy[0] / xy_scale,
                self.uav_xy[1] / xy_scale,
                config.uav_height_m / z_scale,
                self.usv_xy[0] / xy_scale,
                self.usv_xy[1] / xy_scale,
                0.0,
                self.uuv_xy[0] / xy_scale,
                self.uuv_xy[1] / xy_scale,
                config.underwater_depth_m / z_scale,
                self.target_xy[0] / xy_scale,
                self.target_xy[1] / xy_scale,
                config.underwater_depth_m / z_scale,
                escape_direction[0],
                escape_direction[1],
                self.path_length_m / (2.0 * xy_scale),
            ],
            dtype=np.float64,
        )

    def metrics(self) -> StepInfo:
        config = self.config
        uav3 = np.array([self.uav_xy[0], self.uav_xy[1], config.uav_height_m], dtype=np.float64)
        usv3 = np.array([self.usv_xy[0], self.usv_xy[1], 0.0], dtype=np.float64)
        uuv3 = np.array([self.uuv_xy[0], self.uuv_xy[1], config.underwater_depth_m], dtype=np.float64)
        pc = uav_usv_connectivity(self.uav_xy, self.usv_xy, config.uav_height_m, config)
        delta_bar = underwater_connectivity(self.usv_xy, self.uuv_xy, config)
        balance_gap = energy_balance_gap(config)
        distance = norm(self.target_xy - self.uuv_xy)
        constraint_ok = (
            config.h_min_m <= config.uav_height_m <= config.h_max_m
            and distance <= uav_search_radius_m(config.uav_height_m, config)
            and pc >= config.uav_usv_pc_min
            and delta_bar >= config.connectivity_c1
            and balance_gap <= config.energy_balance_c2
        )
        return StepInfo(
            distance_m=distance,
            path_length_m=self.path_length_m,
            energy_kj=motion_energy_kj(self.path_length_m, config),
            uav_usv_distance_m=norm(uav3 - usv3),
            usv_uuv_distance_m=norm(usv3 - uuv3),
            pc=pc,
            delta_bar=delta_bar,
            energy_balance_gap=balance_gap,
            success=distance <= config.safe_radius_m,
            constraint_ok=constraint_ok,
        )

    def step(self, action: int) -> tuple[np.ndarray, float, bool, StepInfo]:
        if self.done:
            return self.observation(), 0.0, True, self.metrics()

        config = self.config
        self.steps += 1
        previous_distance = norm(self.target_xy - self.uuv_xy)

        direction = ACTION_DIRECTIONS[int(action) % self.action_size]
        uuv_delta = direction * config.uuv_speed_mps * config.time_step_s
        next_uuv_xy = clamp_xy(self.uuv_xy + uuv_delta, config.area_size)
        self.path_length_m += norm(next_uuv_xy - self.uuv_xy)
        self.uuv_xy = next_uuv_xy

        escape_direction = unit(self.target_xy - self.uuv_xy)
        self.target_xy = clamp_xy(
            self.target_xy + escape_direction * config.target_speed_mps * config.time_step_s,
            config.area_size,
        )

        relay_goal = relay_target_xy(self.uav_xy, self.uuv_xy, config)
        usv_delta = relay_goal - self.usv_xy
        max_usv_step = config.usv_speed_mps * config.time_step_s
        if norm(usv_delta) > max_usv_step:
            usv_delta = unit(usv_delta) * max_usv_step
        self.usv_xy = clamp_xy(self.usv_xy + usv_delta, config.area_size)

        info = self.metrics()
        if info.success:
            reward = config.terminal_reward
            self.done = True
        elif not info.constraint_ok:
            reward = config.negative_reward
            self.done = True
        elif info.distance_m < previous_distance:
            reward = config.positive_reward
        else:
            reward = config.negative_reward

        if self.steps >= config.max_steps:
            self.done = True

        self.previous_distance_m = info.distance_m
        return self.observation(), reward, self.done, info
