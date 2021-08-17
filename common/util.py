import math


STEP_COUNT = 'step_count'


def reward(state) -> float:
    return math.exp(-math.cos(state.pole_angle))
