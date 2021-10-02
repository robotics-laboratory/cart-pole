import logging
import time
from math import pi

import numpy as np
import scipy.linalg as linalg

from interface import State


class Actor:
    '''Base class for cartpole control algorithms'''

    def __init__(self, **_):
        '''Implement this to use extra params (kwargs) for your algorithm'''
        pass

    def __call__(self, state: State) -> float:
        raise NotImplementedError


class OscillatingActor(Actor):
    '''Sample actor that's trying to oscillate in given X range'''

    def __init__(self, device_config, acceleration, max_position, **_):
        self.direction = True  # Right
        self.a = acceleration
        self.max_x = max_position
        self.max_v = device_config.max_velocity

    def __call__(self, state: State):
        if self.direction:
            if state.position < self.max_x:
                return self.a if state.velocity < self.max_v else 0
            else:
                self.direction = False
                return self(state)
        else:
            if state.position > -self.max_x:
                return -self.a if -state.velocity < self.max_v else 0
            else:
                self.direction = True
                return self(state)



class LQR:
    '''
    Infinite-horizon, continuous-time linear–quadratic regulator.
    Evolution of system: x' = A*x + B*u
    Quadratic cost function: J = I[0, inf](x^T*Q*x + u^T*R*u dt)
    More info:
    - https://en.wikipedia.org/wiki/Linear–quadratic_regulator
    - https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
    '''

    def __init__(self, A, B, Q, R):
        '''
        Create regulator. K matrix is cached.
        '''

        P = linalg.solve_continuous_are(A, B, Q, R)
        self.K = -linalg.inv(R) @ (B.T @ P)

    def __call__(self, x):
        '''
        Returns control u for current state x.
        '''
        return self.K @ x


class LinearBalanceControl(Actor):
    def __init__(self, gravity, pole_length, eps, countdown, **_):
        super().__init__()
        g = gravity
        l = pole_length

        A = np.array(
            [
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, 0, 0, 0],
                [0, 3 * g / (2 * l), 0, 0],
            ]
        )

        B = np.array(
            [
                [0],
                [0],
                [1],
                [3 / (2 * l)],
            ]
        )

        Q = np.zeros((4, 4))
        Q[0, 0] = 1
        Q[1, 1] = 100
        Q[2, 2] = 0
        Q[3, 3] = 0

        R = np.zeros((1, 1))
        R[0, 0] = 1

        self.lqr = LQR(A, B, Q, R)
        self.eps = eps
        self.countdown = countdown
        self.countdown_start = None
        self.activated = False
        self.logger = logging.getLogger('LinearBalanceControl')

    def __call__(self, state: State) -> float:
        '''
        Return desired acceleration for current state.
        '''
        stabilized = abs(state.pole_angle - pi) < self.eps
        if not self.activated:
            if stabilized:
                if self.countdown_start is None:
                    self.countdown_start = time.perf_counter()
                elif (time.perf_counter() - self.countdown_start) > self.countdown:
                    self.activated = True
                    return self(state)
            return 0
        elif not stabilized:
            raise RuntimeError("ABORT")

        error = np.array(
            [
                state.position,
                state.pole_angle - pi,
                state.velocity,
                state.pole_angular_velocity,
            ]
        )

        u = self.lqr(error)[0]
        self.logger.debug('error=%s, u=%s', error, u)
        return u
