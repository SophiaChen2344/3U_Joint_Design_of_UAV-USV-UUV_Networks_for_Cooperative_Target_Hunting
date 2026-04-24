"""Placeholder critic network for DDPG."""


class Critic:
    def __init__(self, state_dim: int) -> None:
        self.state_dim = state_dim

    def __call__(self, _state, _action):
        raise NotImplementedError("Implement Q(s, a | Theta_Q).")
