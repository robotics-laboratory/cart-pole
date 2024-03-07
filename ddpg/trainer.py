from cartpole.common import State, BaseModel
from replay_memory import ReplayMemory, state_to_tensor
from models import Actor, Critic

import torch
import numpy as np

class Scalar(BaseModel):
    value: float

class Config(BaseModel):
    state_dim: int
    action_dim: int
    max_action: float
    discount: float
    memory_size: int
    device_name: str 
    batch_size: int
    actor_lr: float
    critic_lr: float
    tau: float


def compute_reward(state: State):
    value = 1-np.cos(state.pole_angle)
    # value -= 10*(abs(state.cart_position) > 0.4)
    return Scalar(value=value)

class Trainer:
    def __init__(self, logger, config):
        self.logger = logger
        self.config = config
        self.device = torch.device(config.device_name)

        self.memory = ReplayMemory(config.state_dim, config.memory_size)
        
        self.actor = Actor(config.state_dim, config.action_dim, config.max_action).to(self.device)
        self.critic = Critic(config.state_dim, config.action_dim).to(self.device)

        self.target_actor = Actor(config.state_dim, config.action_dim, config.max_action).to(self.device)
        self.target_critic = Critic(config.state_dim, config.action_dim).to(self.device)

        self.target_actor.load_state_dict(self.actor.state_dict())
        self.target_critic.load_state_dict(self.critic.state_dict())

        self.target_actor.eval()
        self.target_critic.eval()

        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=config.actor_lr)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=config.critic_lr)

        self.loss = torch.nn.MSELoss()

    def soft_update(self):
        self.do_soft_update(self.actor, self.target_actor)
        self.do_soft_update(self.critic, self.target_critic)

    def do_soft_update(self, source_net, target_net):
        for source_param, target_param in zip(source_net.parameters(), target_net.parameters()):
            target_param.data = self.config.tau * source_param.data + (1 - self.config.tau) * target_param.data
    
    
    def select_action(self, state: State, sigma: float, train: bool):
        with torch.no_grad():
            self.actor.eval()
            act = self.actor(state_to_tensor(state).reshape(1, -1).to(self.device))
            self.actor.train()

            self.critic.eval()
            rew1, rew2 = self.target_critic(state_to_tensor(state).reshape((1, -1)).to(self.device), act)
            self.logger.publish('/cartpole/est_reward1', Scalar(value=rew1.cpu().numpy()), state.stamp)
            self.logger.publish('/cartpole/est_reward2', Scalar(value=rew2.cpu().numpy()), state.stamp)
            self.logger.publish('/cartpole/action', Scalar(value=act.cpu().numpy()), state.stamp)
            self.critic.train()

        noise = 0.0

        if train:
            noise = torch.normal(0.0, sigma, size=act.shape).to(self.device)

        noisy_action = act + noise
        noisy_action = noisy_action.clip(min=-self.config.max_action, max=self.config.max_action)

        return noisy_action

    def learn(self, total):
        if self.memory.length < self.config.batch_size:
            return

        states, next_states, actions, rewards, dones = self.memory.sample(self.config.batch_size, self.device)

        with torch.no_grad():
            next_actions = self.target_actor(next_states)
            action_noise = torch.normal(0, 0.05, size=next_actions.shape).to(self.device)
            target_Q1, target_Q2 = self.target_critic(next_states, next_actions + action_noise)
            mask = target_Q2.abs() > target_Q2.abs()
            target_Q = target_Q1*mask + target_Q2*(~mask)
            target_Q = rewards + self.config.discount * target_Q

        current_Q1, current_Q2 = self.critic(states, actions)
        critic_loss = self.loss(current_Q1, target_Q) + self.loss(current_Q2, target_Q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 0.1)
        self.critic_optimizer.step()

        if total % 3 == 0:
            actor_loss = -self.critic.Q1(states, self.actor(states)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 0.1)
            self.actor_optimizer.step()
            self.soft_update()