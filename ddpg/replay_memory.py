from cartpole import State, Error

import random
import torch

from math import pi

def state_to_tensor(state: State):
    return torch.tensor([
        state.cart_position,
        state.cart_velocity,
        state.cart_acceleration,
        state.pole_angle,
        state.pole_angular_velocity,
    ])

def make_tensor(from_state, to_state, action, reward):
    return torch.concat([
        state_to_tensor(from_state),
        state_to_tensor(to_state),
        torch.tensor([action]),
        torch.tensor([reward]),
        torch.tensor([to_state.error != Error.NO_ERROR])
    ])

class ReplayMemory:
    def __init__(self, state_size: int, maxlen: int):
        self.state_size = state_size
        self.states = torch.zeros((maxlen, self.state_size*2 + 3))
        self.maxlen = maxlen
        self.ptr = 0
        self.length = 0

    def add(self, from_state: State, to_state: State, action: float, reward: float):
        self.states[self.ptr] = make_tensor(from_state, to_state, action, reward)
        self.ptr = (self.ptr + 1) % self.maxlen
        self.length = min(self.length + 1, self.maxlen)
    
    def sample(self, sample_size: int):
        sample_size = min(self.length, sample_size)
        sample = self.states[random.sample(range(self.length), sample_size)]
        return (
            sample[:, :self.state_size],
            sample[:, self.state_size:self.state_size*2],
            sample[:, self.state_size*2].reshape(-1, 1),
            sample[:, self.state_size*2+1].reshape(-1, 1),
            sample[:, self.state_size*2+2].reshape(-1, 1)
        )