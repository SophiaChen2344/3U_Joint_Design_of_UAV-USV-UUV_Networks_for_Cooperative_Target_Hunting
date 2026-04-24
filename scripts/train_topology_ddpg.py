from __future__ import annotations

from pathlib import Path
import sys
import random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.nn.functional import mse_loss
from collections import deque

# 路径配置
ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from uav_repro.config import load_json

# 设备配置（优先GPU）
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ====================== 1. 实现Actor网络（策略网络） ======================
class Actor(nn.Module):
    """DDPG Actor网络：输入状态，输出确定性动作"""
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()  # 动作归一化到[-1,1]（可根据任务调整）
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.net(state)

# ====================== 2. 实现Critic网络（价值网络） ======================
class Critic(nn.Module):
    """DDPG Critic网络：输入状态+动作，输出动作价值Q(s,a)"""
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)  # 输出单个Q值
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        x = torch.cat([state, action], dim=1)  # 拼接状态和动作
        return self.net(x)

# ====================== 3. 实现DDPG Agent（包含update方法） ======================
class DDPGAgent:
    def __init__(self, state_dim: int, action_dim: int, cfg: dict):
        # 超参数
        self.gamma = cfg["gamma"]  # 折扣因子
        self.tau = cfg["tau"]      # 目标网络软更新系数
        self.lr_actor = cfg["lr_actor"]
        self.lr_critic = cfg["lr_critic"]
        self.memory_capacity = cfg["memory_capacity"]
        self.batch_size = cfg["batch_size"]

        # 经验回放池
        self.memory = deque(maxlen=self.memory_capacity)

        # 主网络
        self.actor = Actor(state_dim, action_dim).to(DEVICE)
        self.critic = Critic(state_dim, action_dim).to(DEVICE)

        # 目标网络（初始与主网络参数一致）
        self.target_actor = Actor(state_dim, action_dim).to(DEVICE)
        self.target_critic = Critic(state_dim, action_dim).to(DEVICE)
        self.target_actor.load_state_dict(self.actor.state_dict())
        self.target_critic.load_state_dict(self.critic.state_dict())

        # 优化器
        self.actor_optim = optim.Adam(self.actor.parameters(), lr=self.lr_actor)
        self.critic_optim = optim.Adam(self.critic.parameters(), lr=self.lr_critic)

    def store_transition(self, state, action, reward, next_state, done):
        """存储经验（s,a,r,s',done）到回放池"""
        self.memory.append((state, action, reward, next_state, done))

    def sample_batch(self):
        """从回放池采样一批经验，并转换为tensor"""
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        # 转换为tensor并移到指定设备
        states = torch.tensor(states, dtype=torch.float32).to(DEVICE)
        actions = torch.tensor(actions, dtype=torch.float32).to(DEVICE)
        rewards = torch.tensor(rewards, dtype=torch.float32).unsqueeze(1).to(DEVICE)
        next_states = torch.tensor(next_states, dtype=torch.float32).to(DEVICE)
        dones = torch.tensor(dones, dtype=torch.float32).unsqueeze(1).to(DEVICE)
        
        return states, actions, rewards, next_states, dones

    def update(self):
        """核心更新方法：更新Actor和Critic网络"""
        # 经验池不足时不更新
        if len(self.memory) < self.batch_size:
            return
        
        # 采样批次数据
        states, actions, rewards, next_states, dones = self.sample_batch()

        # --------------------- 1. 更新Critic网络 ---------------------
        # 目标Q值：r + γ * Q_target(s', a_target(s')) * (1 - done)
        with torch.no_grad():
            next_actions = self.target_actor(next_states)
            target_q = self.target_critic(next_states, next_actions)
            target_q = rewards + self.gamma * target_q * (1 - dones)
        
        # 当前Q值
        current_q = self.critic(states, actions)
        
        # Critic损失（MSE）
        critic_loss = mse_loss(current_q, target_q)
        self.critic_optim.zero_grad()
        critic_loss.backward()
        self.critic_optim.step()

        # --------------------- 2. 更新Actor网络 ---------------------
        # Actor损失（最大化Q值，等价于最小化 -Q(s, a_pred)）
        pred_actions = self.actor(states)
        actor_loss = -self.critic(states, pred_actions).mean()
        
        self.actor_optim.zero_grad()
        actor_loss.backward()
        self.actor_optim.step()

        # --------------------- 3. 软更新目标网络 ---------------------
        self._soft_update(self.target_actor, self.actor, self.tau)
        self._soft_update(self.target_critic, self.critic, self.tau)

    def _soft_update(self, target_net: nn.Module, source_net: nn.Module, tau: float):
        """软更新：target = tau*source + (1-tau)*target"""
        for target_param, source_param in zip(target_net.parameters(), source_net.parameters()):
            target_param.data.copy_(tau * source_param.data + (1 - tau) * target_param.data)

    def get_action(self, state: np.ndarray, noise: float = 0.1) -> np.ndarray:
        """获取动作（带探索噪声）"""
        state = torch.tensor(state, dtype=torch.float32).unsqueeze(0).to(DEVICE)
        with torch.no_grad():
            action = self.actor(state).squeeze(0).cpu().numpy()
        # 添加高斯噪声（探索）
        action += noise * np.random.randn(*action.shape)
        # 裁剪动作到[-1,1]
        action = np.clip(action, -1, 1)
        return action

# ====================== 4. 完善训练主逻辑 ======================
def main() -> None:
    # 加载配置
    cfg = load_json(ROOT / "configs" / "ddpg.json")
    print("DDPG training scaffold loaded.")
    print(f"Episodes: {cfg['episodes']}")

    # 假设的状态/动作维度（需根据UAV拓扑任务调整！）
    # 请替换为你任务的实际维度
    STATE_DIM = cfg.get("state_dim", 10)   # UAV状态维度（例：位置、拓扑连接、干扰等）
    ACTION_DIM = cfg.get("action_dim", 3)  # 动作维度（例：UAV移动方向、拓扑调整指令等）

    # 初始化Agent
    agent = DDPGAgent(STATE_DIM, ACTION_DIM, cfg)

    # 训练循环
    for episode in range(cfg["episodes"]):
        # 重置环境（需替换为你的UAV环境重置逻辑）
        state = np.random.rand(STATE_DIM)  # 示例：随机初始状态
        episode_reward = 0
        done = False

        while not done:
            # 1. 获取动作
            action = agent.get_action(state)
            
            # 2. 执行动作（需替换为你的UAV环境step逻辑）
            # 示例：模拟环境反馈
            next_state = np.random.rand(STATE_DIM)
            reward = np.random.uniform(-1, 1)  # 示例：随机奖励（需替换为任务真实奖励）
            done = random.random() < 0.1       # 示例：随机结束（需替换为任务终止条件）
            
            # 3. 存储经验
            agent.store_transition(state, action, reward, next_state, done)
            
            # 4. 更新Agent
            agent.update()
            
            # 5. 状态更新
            state = next_state
            episode_reward += reward

        # 打印训练进度
        if (episode + 1) % 10 == 0:
            print(f"Episode [{episode+1}/{cfg['episodes']}], Reward: {episode_reward:.2f}")

    print("DDPG training completed!")

if __name__ == "__main__":
    main()