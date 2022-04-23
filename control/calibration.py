from common import Config, State, generate_pyplot_animation
from control import BalanceLQRControl, Trajectory, TrajectoryLQRControl
from simulator import CartPoleSimulator

import matplotlib.pyplot as plt
import numpy as np

from IPython.display import HTML

"""
Calibration steps!

1. TRACK'S LENGTH *update*
    1. Move max to the right
    2. Move max to the left
    3. *update* hard_max_position = covered distance
    4. *update* max_position = 0.9 * hard_max_position
    5. *update* zero_point (in State?)

2. HOMING

3. POLE'S MASS AND LENGTH *update*
    1. Some sin-looking accelerated trajectory 
    2. *update* pole_mass
    3. *update* pole_length
    
4. HOMING
"""


class Calibration:
    
    max_position_calibr: float = 0  # m
    hard_max_position_calibr: float = 0  # m
        
    pole_length_calibr: float = 0  # m
    pole_mass_calibr: float = 0  # kg

    
    def __init__(self, config):
        
        
        
    def __call__(self, state):
        
        return 
    