import time
from common import Config, State, generate_pyplot_animation
from simulator import CartPoleSimulator
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
from IPython.display import HTML


class Calibrator:
    def __init__(self, config: Config, cart_pole):
        self.config = config
        self.cart_pole = cart_pole

        self.move_index = 0
        self.moves_num = 0
        self.acceleration = 0
        self.length = 0
        self.zero_angle = 0
        self.last_acc_change_timestamp = None
        self.first_acc_duration = None
        self.start_timestamp = None
        self.data = []

    def __braking_distance(self):
        dist = -0.02 * self.data[-1]["state"].cart_velocity
        return dist

    def __pos_turned_positive(self):
        return (self.data[-1]["state"].cart_position >= self.__braking_distance()) \
               and (self.data[-4]["state"].cart_position <= self.__braking_distance()) \
               and (self.last_acc_change_timestamp + 0.15 < self.data[-1]["timestamp"])

    def __pos_turned_negative(self):
        return (self.data[-1]["state"].cart_position <= self.__braking_distance()) \
               and (self.data[-4]["state"].cart_position >= self.__braking_distance()) \
               and (self.last_acc_change_timestamp + 0.15 < self.data[-1]["timestamp"])

    def __reset(self):
        self.cart_pole.reset(self.config)  # todo: update config's max_position
        time.sleep(3.0)

        self.length = self.config.max_position * 0.75
        self.acceleration = (min(self.config.max_acceleration, (self.config.max_velocity ** 2) / (2 * self.length))
                             / (self.moves_num + 1)) * 0.4
        self.first_acc_duration = (self.length / self.acceleration) ** 0.5
        self.zero_angle = self.cart_pole.get_state().pole_angle
        self.start_timestamp = self.cart_pole.timestamp()

    def __collect_data(self):
        target = self.acceleration
        timestamp = self.cart_pole.timestamp() - self.start_timestamp
        self.last_acc_change_timestamp = timestamp - self.start_timestamp

        while timestamp < self.first_acc_duration:
            self.cart_pole.set_target(target)
            state = self.cart_pole.get_state()
            state.pole_angle -= self.zero_angle
            timestamp = self.cart_pole.timestamp() - self.start_timestamp
            self.data.append({"state": state, "timestamp": timestamp, "target": target})

        target = -target

        while self.move_index < self.moves_num:
            self.cart_pole.set_target(target)
            state = self.cart_pole.get_state()
            state.pole_angle -= self.zero_angle
            timestamp = self.cart_pole.timestamp() - self.start_timestamp
            self.data.append({"state": state, "timestamp": timestamp, "target": target})

            if self.__pos_turned_negative():
                self.move_index += 1
                target = abs(target)
                self.last_acc_change_timestamp = timestamp

            if self.__pos_turned_positive():
                target = (-abs(target)) * ((self.move_index + 1) / self.move_index)
                self.last_acc_change_timestamp = timestamp

    def __update_params(self):
        n = len(self.data)

        epsilon = []
        expression = []  # expression: -((x¨)cos(θ) + g*sin(θ))

        for i in range(1, n):
            curr_target = self.data[i]["target"]
            curr_state = self.data[i]["state"].as_tuple()
            prev_state = self.data[i - 1]["state"].as_tuple()
            time_step = self.data[i]["timestamp"] - self.data[i - 1]["timestamp"]

            expression.append(-((curr_target * cos(curr_state[1])) + (self.config.gravity * sin(curr_state[1]))))
            epsilon.append((curr_state[3] - prev_state[3]) / time_step)

        expression = np.array(expression)
        epsilon = np.vstack([epsilon, np.zeros(len(epsilon))]).T
        self.config.pole_length = np.linalg.lstsq(epsilon, expression)[0][0]

    def calibrate(self, moves_num=10):
        self.moves_num = moves_num

        self.__reset()
        self.__collect_data()
        self.cart_pole.reset(self.config)
        self.__update_params()
        self.config.pole_length *= 1.18
        print(self.config.pole_length)


        return self.config
