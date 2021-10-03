import logging
import math
import os

from common.interface import State

STEP_COUNT = 'step_count'


def reward(state: State) -> float:
    return math.exp(-math.cos(state.pole_angle))


def init_logging():
    '''
    Helper function to initialize logging. In top-level scripts (__name__ == __main__)
    you should call this function as soon as possible, to ensure no logs will be printed
    before logging is initialized. Only first call of this function is effective - all
    later calls are ignored.

    Logging options are customizable via environment variables:
    * LOGGING_FORMAT - logging format string (default: see below)
    * LOGGING_LEVEL - logging level (default: INFO)
    '''
    DEFAULT_FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'

    if getattr(init_logging, '__initialized', False):
        return  # Already initialized

    format = os.environ.get('LOGGING_FORMAT', DEFAULT_FORMAT)
    level = getattr(logging, os.environ.get('LOGGING_LEVEL', 'INFO').upper())
    logging.basicConfig(format=format, level=level)
    setattr(init_logging, '__initialized', True)
