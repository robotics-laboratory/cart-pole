#!/usr/bin/env python
# coding: utf-8


import asyncio
import collections
import json
import sys
import random
import time
import torch

import matplotlib.pyplot as plt

from copy import deepcopy
from dataclasses import dataclass
from torch import FloatTensor
from typing import Collection
from tqdm import tqdm

from cartpole import LogServer, State, Error

# System parameters

@dataclass(init=False)
@dataclass
class Limits:
    position: float = 0.25
    velocity: float = 3.0
    angle: float = 2 * torch.pi * 2 # no more 2 rotations 
    acceleration: float = 2.5

@dataclass(init=False)
class DynamicSystemConfig:
    gravity: float = 9.8
    pole_length: float = 0.3
    limits = Limits()

# Simple simulation based on dynamics

@dataclass(init=False)
class SimulatorConfig:
    integration_interval: float = 0.02
    integration_step_n: int = 20

    @property
    def time_delta(self) -> float:
        return self.integration_interval / self.integration_step_n

class Simulator:
    def __init__(self, dynamics = DynamicSystemConfig(), config = SimulatorConfig()):
        self.dynamics = dynamics
        self.config = config

    def derivative(self, s: FloatTensor, a: FloatTensor) -> FloatTensor:
        result = torch.empty_like(s)
        theta = s[1]

        g = self.dynamics.gravity
        l = self.dynamics.pole_length

        result[0] = s[2]
        result[1] = s[3]
        result[2] = a
        result[3] = -1.5 / l * (a * torch.cos(theta) + g * torch.sin(theta))

        return result


    def advance(self, x: FloatTensor, a: FloatTensor) -> FloatTensor:
        x = x.clone()
        dt = self.config.time_delta

        for _ in range(self.config.integration_step_n):
            ds1 = self.derivative(x, a)
            ds2 = self.derivative(x + ds1 * dt, a)
            x += (ds1 + ds2) / 2 * dt
        return x

# Environment

class Environment:
    def __init__(self, dynamics = DynamicSystemConfig(), config = SimulatorConfig()):
        self.dynamics = dynamics
        self.simulator = Simulator(dynamics, config)

    def is_finish(self, s: FloatTensor) -> bool:
        return s[0].abs() >= self.dynamics.limits.position \
            or s[2].abs() >= self.dynamics.limits.velocity \
            or s[1].abs() >= self.dynamics.limits.angle


    def reset(self) -> FloatTensor:
        s = torch.zeros(4, 1)
        # s[1].uniform_(-torch.pi + 1e-3, torch.pi - 1e-3)
        # s[1].normal_(torch.pi, 0.1)
        s[1].normal_(0.0, 0.2)
        return s

    def reward(self, s: FloatTensor, a: FloatTensor) -> float:
        angle_reward = torch.exp(1-torch.cos(s[1])) + (1 - s[3].abs() / self.dynamics.limits.angle) - 0.26
        return angle_reward

    def advance(self, s: FloatTensor, a: FloatTensor) -> FloatTensor:
        s_next = self.simulator.advance(s, a)
        return s_next, self.reward(s_next, a), self.is_finish(s_next)


# Replay buffer

@dataclass
class Transition:
    state: FloatTensor
    action: FloatTensor
    next_state: FloatTensor
    reward: float
    is_finish: bool

class ReplayBuffer:
    def __init__(self, capacity: int):
        self.capacity = capacity
        self.buffer = collections.deque()

    def push(self, transition):
        self.buffer.append(transition)
        if len(self.buffer) > self.capacity:
            self.buffer.popleft()

    def __len__(self):
        return len(self.buffer)

    def sample(self, batch_n: int) -> Collection[FloatTensor]:
        batch = random.sample(self.buffer, batch_n)
        s = torch.hstack([t.state for t in batch])
        s_next = torch.hstack([t.next_state for t in batch])

        a = torch.tensor([t.action for t in batch])
        r = torch.tensor([t.reward for t in batch])
        finish = torch.tensor([t.is_finish for t in batch])

        return s, a, s_next, r, finish
 

class Actor(torch.nn.Module):
    def __init__(self, limits: Limits):
        super(Actor, self).__init__()

        self.temprature = 0.01
        self.limits = limits

        self.l1 = torch.nn.Linear(6, 64)
        self.l2 = torch.nn.Linear(64, 256)
        self.l3 = torch.nn.Linear(256, 1)

    def forward(self, x):
        _, batch_n = x.shape
        f = FloatTensor(6, batch_n)

        f[0] = x[0] / self.limits.position
        f[1] = x[2] / self.limits.velocity
        f[2] = x[3] / (2*torch.pi)

        f[3] = x[1] / self.limits.angle
        f[4] = torch.cos(x[1])
        f[5] = torch.sin(x[1])

        f = f.T
        f = self.l1(f)
        f = torch.tanh(f)
        f = self.l2(f)
        f = torch.tanh(f)
        f = self.l3(f)
        a = torch.tanh(self.temprature * f)

        return self.limits.acceleration * a.flatten()


class Critic(torch.nn.Module):
    def __init__(self,limits: Limits):
        super(Critic, self).__init__()

        self.limits = limits

        self.layers = torch.nn.Sequential(
            torch.nn.Linear(7, 64),
            torch.nn.Tanh(),
            torch.nn.Linear(64, 256),
            torch.nn.Tanh(),
            torch.nn.Linear(256, 1),
        )

    def forward(self, x, a):
        _, batch_n = x.shape
        features = FloatTensor(7, batch_n)

        features[0] = x[0] / self.limits.position
        features[1] = x[2] / self.limits.velocity
        features[2] = x[3] / (2*torch.pi)
        features[3] = a / self.limits.acceleration
       
        features[4] = x[1] / (2*torch.pi)
        features[5] = torch.cos(x[1])
        features[6] = torch.sin(x[1])

        q_value = self.layers(features.T)
        return q_value.flatten()

session_n = 10000
max_session_size = 2000
gamma = 0.99
batch_n = 128
tau = 0.99
decay = 0.99
sigma = 0.2
lr = 1e-3
buf_size = 10 * max_session_size

foxglove = LogServer(log_path='log.mcap')

dynamic_system_config = DynamicSystemConfig()
simulator_config = SimulatorConfig()

cart_pole = Environment(dynamic_system_config, simulator_config)
buffer = ReplayBuffer(20000)

actor = Actor(dynamic_system_config.limits)
actor_target = Actor(dynamic_system_config.limits)
actor_optimizer = torch.optim.Adam(actor.parameters(), lr=lr)


critic = Critic(dynamic_system_config.limits)
critic_target = Critic(dynamic_system_config.limits)
critic_optimizer = torch.optim.Adam(critic.parameters(), lr=lr)

def mutate(current_net, target_net):
    current_net_dict = current_net.state_dict()
    target_net_dict = target_net.state_dict()
    
    for key in target_net_dict:
        target_net_dict[key] = (1 - tau) * current_net_dict[key] + tau * target_net_dict[key]
        target_net.load_state_dict(target_net_dict)

def update():
    s, a, s_next, r, finish = buffer.sample(batch_n)

    q_value = critic(s, a)
    q_value_next = critic_target(s_next, actor_target(s_next)).detach()
    q_value_next[finish] = 0

    q_target = r + gamma * q_value_next

    critic_loss = torch.nn.functional.mse_loss(q_value, q_target)
    critic_optimizer.zero_grad()
    critic_loss.backward()
    critic_optimizer.step()

    actor_loss = -critic(s, actor(s)).mean()
    actor_optimizer.zero_grad()
    actor_loss.backward()
    actor_optimizer.step()

    return critic_loss, -actor_loss

print("Init replay buffer...")
for _ in tqdm(range(30)):
    s = cart_pole.reset()
    a = torch.tensor([0.0])
    for _ in range(100):
        s_next, r, finish = cart_pole.advance(s, a)
        buffer.push(Transition(s, a,s_next, r, finish))

print('Training...')

counter = 0
for _ in tqdm(range(session_n)):
    s = cart_pole.reset()

    sigma *= decay

    for step in range(max_session_size):
        with torch.no_grad():
            a = actor(s).detach()# + torch.randn(1) * sigma
        s_next, r, finish = cart_pole.advance(s, a)

        if step + 1 == max_session_size:
            finish = True

        buffer.push(Transition(s, a, s_next, r, finish))
        foxglove.publish_state(
            State(s[0].item(), s[2].item(), s[1].item(), s[3].item(), Error.NO_ERROR, a.item()))
        s = s_next.detach()

        critic_loss, actor_critic_reward = update()
        mutate(actor, actor_target)
        mutate(critic, critic_target)

        foxglove.publish_actor_critic(actor_critic_reward.item(), critic_loss.item(), r.item())

        counter += 1
        if finish:
            break

