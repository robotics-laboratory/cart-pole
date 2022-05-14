from common import Config, State
from simulator import CartPoleSimulator
import numpy as np
import random
from math import sin, cos


def acc_change_duration(a0, l0, moves_num):
    t0 = (l0 / a0) ** 0.5
    
    change_timestamps = [t0]
    
    for i in range(moves_num):
        timestamp = t0 * ((2 / (i + 2)) ** 0.5)
        change_timestamps.append(timestamp) 
    
    return change_timestamps


def collect_data(config, cart_pole, moves_num):
    a0 = a = config.max_acceleration / ((moves_num + 1) * 1.1)
    l0 = config.max_position * 0.9
    change_timestamps = acc_change_duration(a0, l0, moves_num)
    
    data = []
    cart_pole.reset(config)
    start_time = cart_pole.timestamp()
    
    while change_timestamps:
        time = cart_pole.timestamp() - start_time
        state = cart_pole.get_state()
        data.append({"state": state, "time": time, "target": a})
        
        if time > change_timestamps[0]:
            a = -a
            change_timestamps.pop(0)
            
        cart_pole.set_target(a)
        timestepp = round(random.uniform(0.008, 0.012), 10)
        cart_pole.advance(timestepp)
        print(time, timestepp, time + timestepp)
        
    return data
     

def execute_parameters(data, config):
    N = len(data)

    epsilon = []
    expression = []  #expression: -((x¨)cos(θ) + g*sin(θ))
    
    for i in range(1, N):
        curr_target = data[i]["target"]
        curr_state = data[i]["state"].as_tuple()
        prev_state = data[i - 1]["state"].as_tuple()
        time_step = data[i]["time"] - data[i - 1]["time"]
        
        expression.append(-((curr_target * cos(curr_state[1])) + (config.gravity * sin(curr_state[1]))))
        epsilon.append((curr_state[3] - prev_state[3]) / time_step)
        
    expression = np.array(expression)
    epsilon = np.vstack([epsilon, np.zeros(len(epsilon))]).T
    pole_length = np.linalg.lstsq(epsilon, expression)[0][0]

    return pole_length


def calibrate(config, cart_pole, moves_num = 18):
    data = collect_data(config, cart_pole, moves_num)
    config.pole_length = execute_parameters(data, config)
    cart_pole.reset(config)
    
    return config, cart_pole