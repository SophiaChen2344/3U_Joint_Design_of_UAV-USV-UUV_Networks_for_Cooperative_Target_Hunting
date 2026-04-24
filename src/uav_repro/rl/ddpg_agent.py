from __future__ import annotations

from dataclasses import dataclass

from .replay_buffer import ReplayBuffer


@dataclass
class DDPGConfig:
    gamma: float
    actor_lr: float
    critic_lr: float
    batch_size: int
    soft_update_tau: float
    replay_buffer_size: int


class DDPGAgent:
    """Training skeleton following Eq. (24)-(31)."""

    def __init__(self, cfg: DDPGConfig, state_dim: int) -> None:
        self.cfg = cfg
        self.state_dim = state_dim
        self.replay_buffer = ReplayBuffer(cfg.replay_buffer_size)

    def select_action(self, _state) -> float:
        raise NotImplementedError("Implement actor inference and exploration noise.")

    def update(self) -> None:
        raise NotImplementedError("Implement Eq. (24)-(31).")
