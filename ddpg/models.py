import torch
from torch import nn
from torch.nn import functional as F

import numpy as np

class Actor(nn.Module):
    """Actor (Policy) Model."""

    def __init__(self, state_size, action_size, max_action, fc1_units=128, fc2_units=128):
        super(Actor, self).__init__()
        self.max_action = max_action
        self.nn = nn.Sequential(
              nn.Linear(state_size, fc1_units),
              nn.BatchNorm1d(fc1_units),
              nn.ReLU(),
              nn.Linear(fc1_units, fc2_units),
              nn.BatchNorm1d(fc2_units),
              nn.ReLU(),
              nn.Linear(fc2_units, action_size),
              nn.Tanh()
        )

    def forward(self, state):
        return self.nn(state) * self.max_action


class Critic(nn.Module):
    def __init__(self, state_size, action_size, fc1_units=128, fc2_units=128):
        super(Critic, self).__init__()
        self.q1 = nn.Sequential(
            nn.Linear(state_size + action_size, fc1_units),
            nn.ReLU(),
            nn.BatchNorm1d(fc1_units),
            nn.Linear(fc1_units, fc2_units),
            nn.ReLU(), 
            nn.BatchNorm1d(fc2_units),
            nn.Linear(fc2_units, 1),
        )

        self.q2 = nn.Sequential(
            nn.Linear(state_size + action_size, fc1_units),
            nn.ReLU(),
            nn.BatchNorm1d(fc1_units),
            nn.Linear(fc1_units, fc2_units),
            nn.ReLU(),
            nn.BatchNorm1d(fc2_units),
            nn.Linear(fc2_units, 1),
        )

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        return self.q1(sa), self.q2(sa)


    def Q1(self, state, action):
        sa = torch.cat([state, action], 1)
        return self.q1(sa)