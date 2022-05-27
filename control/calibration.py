import time
from common import Config, State
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
from scipy import interpolate, integrate


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
        dist = -0.018 * self.data[-1]["state"].cart_velocity
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
                             / (self.moves_num + 1)) * 0.5
        self.first_acc_duration = (self.length / self.acceleration) ** 0.5
        self.zero_angle = self.cart_pole.get_state().pole_angle

    def __collect_data(self):
        target = self.acceleration
        self.start_timestamp = self.cart_pole.timestamp()
        timestamp = self.start_timestamp - self.start_timestamp
        state = self.cart_pole.get_state()
        self.data.append({"state": state, "timestamp": timestamp, "target": 0})
        self.last_acc_change_timestamp = timestamp

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
        expression = []  # expression: -((x¨)cos(θ) + g*sin(θ))
        timestamps = []
        thetas = []
        psi_thetas = []
        psi_omegas = []

        for i in range(n):
            expression.append(-((self.data[i]["target"] * cos(self.data[i]["state"].pole_angle))
                                + (self.config.gravity * sin(self.data[i]["state"].pole_angle))))
            timestamps.append(self.data[i]["timestamp"])
            thetas.append(self.data[i]["state"].pole_angle)

        psi_epsilon_func = interpolate.interp1d(np.array(timestamps), np.array(expression))#, kind="cubic")


        for i in range(n):
            timestamp = timestamps[i]
            psi_omegas.append(integrate.quad(lambda t: psi_epsilon_func(t), 0, timestamp)[0])

        psi_omega_func = interpolate.interp1d(np.array(timestamps), np.array(psi_omegas))#, kind="cubic")


        for i in range(n):
            timestamp = timestamps[i]
            psi_thetas.append(integrate.quad(lambda t: psi_omega_func(t), 0, timestamp)[0])



        expression = np.array(expression)
        thetasx = thetas
        thetas = np.vstack([thetas, np.zeros(len(thetas))]).T
        self.config.pole_length = np.linalg.lstsq(thetas, psi_thetas)[0][0]
        print(self.config.pole_length)
        plt.figure(figsize=(60, 40))

        plt.plot(timestamps, psi_thetas, '.', timestamps, thetasx, 'ys')
        plt.show()

    def calibrate(self, moves_num=10):
        self.moves_num = moves_num

        self.__reset()
        self.__collect_data()
        self.cart_pole.reset(self.config)
        self.__update_params()

        return self.config
