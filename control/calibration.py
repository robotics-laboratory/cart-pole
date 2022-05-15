from common import Config, State
from simulator import CartPoleSimulator
import numpy as np
import random
from math import sin, cos


def acc_change_duration(a0, l0, moves_num):
    t0 = (l0 / a0) ** 0.5
    
    sign_change_timestamps = [t0]
    abs_change_timestamps = [2*t0]
    
    for i in range(moves_num):
        timestamp = t0 * ((2 / (i + 2)) ** 0.5)
        sign_change_timestamp = abs_change_timestamps[-1] + timestamp
        abs_change_timestamp = sign_change_timestamps[-1] + timestamp
        sign_change_timestamps.append(sign_change_timestamp)
        abs_change_timestamps.append(abs_change_timestamp)
    
    return sign_change_timestamps, abs_change_timestamps


def collect_data(config, cart_pole, moves_num):
    a0 = a = config.max_acceleration / ((moves_num + 1) * 1.1)
    l0 = config.max_position * 0.7
    sign_change_timestamps, abs_change_timestamps = acc_change_duration(a0, l0, moves_num)
    
    data = []
    k = 1
    # cart_pole.reset(config)
    start_time = cart_pole.timestamp()
    
    while abs_change_timestamps:
        time = cart_pole.timestamp() - start_time
        state = cart_pole.get_state()
        data.append({"state": state, "time": time, "target": a})

        if time > abs_change_timestamps[0]:
            a *= (k + 1) / k
            k += 1
            abs_change_timestamps.pop(0)

        elif time > sign_change_timestamps[0]:
            a = -a
            sign_change_timestamps.pop(0)
            
        cart_pole.set_target(a)
        
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
    # cart_pole.reset(config)
    
    return config.pole_length


if __name__ == '__main__':
    from common.util import init_logging
    from device import CartPoleDevice
    from sessions.collector import CollectorProxy
    from pathlib import Path
    import time
    import logging
    init_logging()
    LOGGER = logging.getLogger('debug-session-runner')

    SESSION_ID = 'calibration_test_1'
    DEVICE_CONFIG = Config(
        max_position=0.25,
        max_velocity=5,
        max_acceleration=10,
        clamp_velocity=True,
        clamp_acceleration=True,
    )

    SESSION_MAX_DURATION = 150.0
    OUTPUT_PATH = Path(f'data/sessions/{SESSION_ID}')
    init_logging()
    device = CartPoleDevice()
    # analyzer = SaleaeAnalyzer()
    proxy = CollectorProxy(
        cart_pole=device,
        actor_class=object,
        actor_config={},
        # reset_callbacks=[analyzer.start],
        # close_callbacks=[analyzer.stop],
    )

    try:
        proxy.reset(DEVICE_CONFIG)
        exit(0)
        time.sleep(5.0)
        # LOGGER.info(">>>")
        # device.interface.set(DeviceTarget(position=0.05))
        # LOGGER.info("<<<")
        ret = calibrate(DEVICE_CONFIG, proxy)
        print(ret)
    except Exception:
        LOGGER.exception('Aborting run due to error')
    finally:
        proxy.close()
        LOGGER.info('Run finished')

    proxy.save(OUTPUT_PATH / 'session.json')
    # analyzer.save(OUTPUT_PATH)
