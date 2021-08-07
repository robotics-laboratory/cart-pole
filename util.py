import math


STEP_COUNT = 'step_count'


def as_bool(s) -> bool:
    if isinstance(s, str):
        return s.lower() == 'true'
    return bool(s)


def reward(state) -> float:
    return math.exp(-math.cos(state.pole_angle))
