from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import random
from typing import Callable

import numpy as np

from .config import PaperConfig
from .environment import ThreeUEnvironment, StepInfo
from .physics import ACTION_DIRECTIONS, clamp_xy, norm, relay_target_xy, unit, uav_search_radius_m


@dataclass
class EpisodeResult:
    reward: float
    steps: int
    success: bool
    path_length_m: float
    energy_kj: float
    uav_usv_distance_m: float
    usv_uuv_distance_m: float
    final_distance_m: float


class ReplayBuffer:
    def __init__(self, capacity: int, rng: random.Random):
        self.storage: deque[tuple[np.ndarray, int, float, np.ndarray, bool]] = deque(maxlen=capacity)
        self.rng = rng

    def append(self, transition: tuple[np.ndarray, int, float, np.ndarray, bool]) -> None:
        self.storage.append(transition)

    def sample(self, batch_size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        batch = self.rng.sample(list(self.storage), batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            np.asarray(states, dtype=np.float64),
            np.asarray(actions, dtype=np.int64),
            np.asarray(rewards, dtype=np.float64),
            np.asarray(next_states, dtype=np.float64),
            np.asarray(dones, dtype=np.float64),
        )

    def __len__(self) -> int:
        return len(self.storage)


class DenseQNetwork:
    def __init__(
        self,
        state_size: int,
        action_size: int,
        rng: np.random.Generator,
        hidden_sizes: tuple[int, int] = (64, 64),
        dueling: bool = False,
    ):
        self.dueling = dueling
        h1, h2 = hidden_sizes
        self.params: dict[str, np.ndarray] = {
            "w1": rng.normal(0.0, math.sqrt(2.0 / state_size), size=(state_size, h1)),
            "b1": np.zeros(h1),
            "w2": rng.normal(0.0, math.sqrt(2.0 / h1), size=(h1, h2)),
            "b2": np.zeros(h2),
        }
        if dueling:
            self.params.update(
                {
                    "wv": rng.normal(0.0, math.sqrt(2.0 / h2), size=(h2, 1)),
                    "bv": np.zeros(1),
                    "wa": rng.normal(0.0, math.sqrt(2.0 / h2), size=(h2, action_size)),
                    "ba": np.zeros(action_size),
                }
            )
        else:
            self.params.update(
                {
                    "w3": rng.normal(0.0, math.sqrt(2.0 / h2), size=(h2, action_size)),
                    "b3": np.zeros(action_size),
                }
            )

    def clone(self) -> "DenseQNetwork":
        copied = object.__new__(DenseQNetwork)
        copied.dueling = self.dueling
        copied.params = {key: value.copy() for key, value in self.params.items()}
        return copied

    def copy_from(self, other: "DenseQNetwork") -> None:
        for key, value in other.params.items():
            self.params[key][...] = value

    def predict(self, states: np.ndarray) -> np.ndarray:
        states = np.atleast_2d(states)
        z1 = states @ self.params["w1"] + self.params["b1"]
        a1 = np.maximum(z1, 0.0)
        z2 = a1 @ self.params["w2"] + self.params["b2"]
        a2 = np.maximum(z2, 0.0)
        if self.dueling:
            values = a2 @ self.params["wv"] + self.params["bv"]
            advantages = a2 @ self.params["wa"] + self.params["ba"]
            return values + advantages - advantages.mean(axis=1, keepdims=True)
        return a2 @ self.params["w3"] + self.params["b3"]

    def train(self, states: np.ndarray, actions: np.ndarray, targets: np.ndarray, learning_rate: float) -> float:
        batch_size = states.shape[0]
        z1 = states @ self.params["w1"] + self.params["b1"]
        a1 = np.maximum(z1, 0.0)
        z2 = a1 @ self.params["w2"] + self.params["b2"]
        a2 = np.maximum(z2, 0.0)

        if self.dueling:
            values = a2 @ self.params["wv"] + self.params["bv"]
            advantages = a2 @ self.params["wa"] + self.params["ba"]
            q_values = values + advantages - advantages.mean(axis=1, keepdims=True)
        else:
            q_values = a2 @ self.params["w3"] + self.params["b3"]

        chosen = q_values[np.arange(batch_size), actions]
        error = (chosen - targets) / batch_size
        loss = float(np.mean((chosen - targets) ** 2))

        grad_q = np.zeros_like(q_values)
        grad_q[np.arange(batch_size), actions] = 2.0 * error

        if self.dueling:
            grad_values = grad_q.sum(axis=1, keepdims=True)
            grad_advantages = grad_q - grad_q.mean(axis=1, keepdims=True)
            grad_wv = a2.T @ grad_values
            grad_bv = grad_values.sum(axis=0)
            grad_wa = a2.T @ grad_advantages
            grad_ba = grad_advantages.sum(axis=0)
            grad_a2 = grad_values @ self.params["wv"].T + grad_advantages @ self.params["wa"].T
            head_grads = {"wv": grad_wv, "bv": grad_bv, "wa": grad_wa, "ba": grad_ba}
        else:
            grad_w3 = a2.T @ grad_q
            grad_b3 = grad_q.sum(axis=0)
            grad_a2 = grad_q @ self.params["w3"].T
            head_grads = {"w3": grad_w3, "b3": grad_b3}

        grad_z2 = grad_a2 * (z2 > 0.0)
        grad_w2 = a1.T @ grad_z2
        grad_b2 = grad_z2.sum(axis=0)
        grad_a1 = grad_z2 @ self.params["w2"].T
        grad_z1 = grad_a1 * (z1 > 0.0)
        grad_w1 = states.T @ grad_z1
        grad_b1 = grad_z1.sum(axis=0)

        grads = {"w1": grad_w1, "b1": grad_b1, "w2": grad_w2, "b2": grad_b2, **head_grads}
        for key, grad in grads.items():
            np.clip(grad, -5.0, 5.0, out=grad)
            self.params[key] -= learning_rate * grad
        return loss


class DQNAgent:
    def __init__(
        self,
        state_size: int,
        action_size: int,
        config: PaperConfig,
        *,
        learning_rate: float | None = None,
        double: bool = False,
        dueling: bool = False,
        seed: int | None = None,
    ):
        self.config = config
        self.action_size = action_size
        self.learning_rate = config.dqn_learning_rate if learning_rate is None else learning_rate
        self.double = double
        self.rng = np.random.default_rng(config.random_seed if seed is None else seed)
        self.py_rng = random.Random(config.random_seed if seed is None else seed)
        self.online = DenseQNetwork(state_size, action_size, self.rng, dueling=dueling)
        self.target = self.online.clone()
        self.replay = ReplayBuffer(config.dqn_memory_capacity, self.py_rng)

    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        if training and self.py_rng.random() < self.config.dqn_paper_epsilon:
            return self.py_rng.randrange(self.action_size)
        return int(np.argmax(self.online.predict(state)[0]))

    def learn(self) -> float | None:
        if len(self.replay) < self.config.dqn_batch_size:
            return None
        states, actions, rewards, next_states, dones = self.replay.sample(self.config.dqn_batch_size)
        if self.double:
            best_actions = np.argmax(self.online.predict(next_states), axis=1)
            next_q = self.target.predict(next_states)[np.arange(next_states.shape[0]), best_actions]
        else:
            next_q = np.max(self.target.predict(next_states), axis=1)
        targets = rewards + (1.0 - dones) * self.config.dqn_discount * next_q
        return self.online.train(states, actions, targets, self.learning_rate)


def run_episode(env: ThreeUEnvironment, policy: Callable[[np.ndarray], int]) -> EpisodeResult:
    state = env.reset()
    total_reward = 0.0
    info = env.metrics()
    done = False
    while not done:
        action = policy(state)
        state, reward, done, info = env.step(action)
        total_reward += reward
    return EpisodeResult(
        reward=total_reward,
        steps=env.steps,
        success=info.success,
        path_length_m=info.path_length_m,
        energy_kj=info.energy_kj,
        uav_usv_distance_m=info.uav_usv_distance_m,
        usv_uuv_distance_m=info.usv_uuv_distance_m,
        final_distance_m=info.distance_m,
    )


def train_dqn(
    config: PaperConfig,
    *,
    episodes: int,
    learning_rate: float | None = None,
    double: bool = False,
    dueling: bool = False,
    seed: int | None = None,
) -> tuple[DQNAgent, list[EpisodeResult]]:
    env = ThreeUEnvironment(config, seed=seed)
    agent = DQNAgent(
        env.state_size,
        env.action_size,
        config,
        learning_rate=learning_rate,
        double=double,
        dueling=dueling,
        seed=seed,
    )
    history: list[EpisodeResult] = []
    update_every = 100
    for episode in range(episodes):
        state = env.reset()
        done = False
        total_reward = 0.0
        info = env.metrics()
        while not done:
            action = agent.select_action(state, training=True)
            next_state, reward, done, info = env.step(action)
            agent.replay.append((state, action, reward, next_state, done))
            agent.learn()
            state = next_state
            total_reward += reward
        if episode % update_every == 0:
            agent.target.copy_from(agent.online)
        history.append(
            EpisodeResult(
                reward=total_reward,
                steps=env.steps,
                success=info.success,
                path_length_m=info.path_length_m,
                energy_kj=info.energy_kj,
                uav_usv_distance_m=info.uav_usv_distance_m,
                usv_uuv_distance_m=info.usv_uuv_distance_m,
                final_distance_m=info.distance_m,
            )
        )
    agent.target.copy_from(agent.online)
    return agent, history


def evaluate_agent(config: PaperConfig, agent: DQNAgent, episodes: int, seed: int | None = None) -> dict[str, float]:
    env = ThreeUEnvironment(config, seed=seed)
    results = [run_episode(env, lambda state: agent.select_action(state, training=False)) for _ in range(episodes)]
    return summarize_results(results)


def candidate_heuristic(env: ThreeUEnvironment, action: int) -> float:
    config = env.config
    direction = ACTION_DIRECTIONS[action]
    candidate_uuv = clamp_xy(env.uuv_xy + direction * config.uuv_speed_mps * config.time_step_s, config.area_size)
    target_direction = unit(env.target_xy - candidate_uuv)
    candidate_target = clamp_xy(
        env.target_xy + target_direction * config.target_speed_mps * config.time_step_s,
        config.area_size,
    )
    current_distance = norm(env.target_xy - env.uuv_xy)
    next_distance = norm(candidate_target - candidate_uuv)
    relay_xy = relay_target_xy(env.uav_xy, candidate_uuv, config)
    relay_penalty = norm(relay_xy - candidate_uuv) / max(config.underwater_link_scale_m, 1e-9)
    search_ok = next_distance <= uav_search_radius_m(config.uav_height_m, config)
    progress = max(current_distance - next_distance, -5.0) + 6.0
    return max(1e-4, progress * math.exp(-0.25 * relay_penalty) * (1.0 if search_ok else 0.05))


def run_aco(config: PaperConfig, *, seed: int | None = None) -> tuple[np.ndarray, EpisodeResult]:
    py_rng = random.Random(config.random_seed if seed is None else seed)
    pheromone = np.ones(len(ACTION_DIRECTIONS), dtype=np.float64)
    best: EpisodeResult | None = None
    best_actions: list[int] = []
    evaporation = config.aco_pheromone_volatility

    for _ in range(config.aco_iterations):
        deposits = np.zeros_like(pheromone)
        for _ in range(config.aco_populations):
            env = ThreeUEnvironment(config, seed=py_rng.randrange(10**9))
            state = env.reset()
            del state
            actions: list[int] = []
            done = False
            info: StepInfo = env.metrics()
            while not done:
                heuristic = np.array([candidate_heuristic(env, action) for action in range(len(ACTION_DIRECTIONS))])
                probabilities = pheromone * heuristic**2
                probabilities = probabilities / probabilities.sum()
                action = int(py_rng.choices(range(len(ACTION_DIRECTIONS)), weights=probabilities, k=1)[0])
                actions.append(action)
                _, _, done, info = env.step(action)
            result = EpisodeResult(
                reward=0.0,
                steps=env.steps,
                success=info.success,
                path_length_m=info.path_length_m,
                energy_kj=info.energy_kj,
                uav_usv_distance_m=info.uav_usv_distance_m,
                usv_uuv_distance_m=info.usv_uuv_distance_m,
                final_distance_m=info.distance_m,
            )
            score = (1_000_000.0 if result.success else 0.0) - result.energy_kj - 10.0 * result.final_distance_m
            best_score = -math.inf if best is None else (1_000_000.0 if best.success else 0.0) - best.energy_kj - 10.0 * best.final_distance_m
            if score > best_score:
                best = result
                best_actions = actions
            if result.success:
                deposit = 1.0 / max(result.path_length_m, 1.0)
                for action in actions:
                    deposits[action] += deposit
        pheromone = (1.0 - evaporation) * pheromone + deposits
        pheromone = np.maximum(pheromone, 1e-6)

    if best is None:
        env = ThreeUEnvironment(config, seed=seed)
        best = run_episode(env, lambda state: int(np.argmax(pheromone)))
    if best_actions:
        action_counts = np.bincount(best_actions, minlength=len(ACTION_DIRECTIONS)).astype(np.float64)
        pheromone += action_counts / max(action_counts.sum(), 1.0)
    return pheromone, best


def summarize_results(results: list[EpisodeResult]) -> dict[str, float]:
    if not results:
        return {}
    successes = [result for result in results if result.success]
    basis = successes if successes else results
    return {
        "episodes": float(len(results)),
        "success_rate": float(sum(result.success for result in results) / len(results)),
        "energy_kj": float(np.mean([result.energy_kj for result in basis])),
        "path_length_m": float(np.mean([result.path_length_m for result in basis])),
        "uav_usv_distance_m": float(np.mean([result.uav_usv_distance_m for result in basis])),
        "usv_uuv_distance_m": float(np.mean([result.usv_uuv_distance_m for result in basis])),
        "steps": float(np.mean([result.steps for result in basis])),
        "final_distance_m": float(np.mean([result.final_distance_m for result in basis])),
    }
