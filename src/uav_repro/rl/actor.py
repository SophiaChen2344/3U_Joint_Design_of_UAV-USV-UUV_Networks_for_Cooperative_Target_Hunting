"""Placeholder actor network for DDPG."""


class Actor:
    def __init__(self, state_dim: int) -> None:
        self.state_dim = state_dim

    def __call__(self, _state):
        raise NotImplementedError("Implement deterministic policy mu(s | Theta_mu).")
