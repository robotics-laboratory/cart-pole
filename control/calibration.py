from common import Config, State
from simulator import CartPoleSimulator
import numpy as np
from math import sin, cos


def acc_change_duration(a0, l0, moves_num):
    t0 = (l0 / a0) ** 0.5
    
    change_timestamps = [t0]
    
    for i in range(moves_num):
        timestamp = t0 * ((2 / (i + 2)) ** 0.5)
        change_timestamps.append(timestamp) 
    
    return change_timestamps


def calibration_utraj(config, moves_num):
    timestamps = []
    targets = []
    
    a = a0 = config.max_acceleration / ((moves_num + 1) * 1.1)
    l0 = config.max_position * 0.9
    time = 0
    
    acc_duration = acc_change_duration(a0, l0, moves_num)
    
    for i in range(moves_num + 1):
        for j in range(int(acc_duration[i] / config.time_step)):
            timestamps.append(time)
            time += config.time_step
            targets.append(a)
            
        a = -a
        
        for j in range(int(acc_duration[i] / config.time_step)):
            timestamps.append(time)
            time += config.time_step
            targets.append(a)
        
        a *= ((i+2)/(i+1))
        
    return timestamps, targets


def collect_data(config, targets):
    simulator = CartPoleSimulator()
    simulator.reset(config)
    states = []
    
    for target in targets:
        simulator.set_target(target)
        state = simulator.get_state()
        states.append(state)
        simulator.advance(config.time_step)
        
    return states  


def execute_parameters(states, targets, config):
    N = len(states)

    epsilon = []
    expression = []  #expression: -((x¨)cos(θ) + g*sin(θ))
    
    for i in range(1, N):
        curr_state = states[i].as_tuple()
        prev_state = states[i - 1].as_tuple()
        
        expression.append(-((targets[i] * cos(curr_state[1])) + (config.gravity * sin(curr_state[1]))))
        epsilon.append((curr_state[3] - prev_state[3]) / config.time_step)
        
    expression = np.array(expression)
    epsilon = np.vstack([epsilon, np.zeros(len(epsilon))]).T
    pole_length = np.linalg.lstsq(epsilon, expression)[0][0]

    return pole_length


def calibrate(config, moves_num = 18):
    timestamps, targets = calibration_utraj(config, moves_num)    
    states = collect_data(config, targets)
    
    return execute_parameters(states, targets, config)