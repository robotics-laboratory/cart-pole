import enum
import numpy
import torch

from pydantic import BaseModel
from typing import Any

class Error(enum.IntEnum):
    NO_ERROR = 0
    NEED_RESET = 1
    CART_POSITION_OVERFLOW = 2
    CART_VELOCITY_OVERFLOW = 3
    CART_ACCELERATION_OVERFLOW = 4
    HARDWARE = 5

    def __bool__(self) -> bool:
        return self != Error.NO_ERROR

class Config(BaseModel):
    # software cart limits
    max_position: float = 0.25  # m
    max_velocity: float = 2.0  # m/s
    max_acceleration: float = 3.5  # m/s^2

class State(BaseModel):
    # cart state
    cart_position: float = 0.0
    cart_velocity: float = 0.0

    # pole state
    pole_angle: float = 0.0
    pole_angular_velocity: float = 0.0

    # control state
    cart_acceleration: float = 0.0
    error: Error = Error.NO_ERROR

    @staticmethod
    def home() -> 'State':
        return State() 


class CartPoleBase:
    '''
    Description:
        The class specifies a interface of the cart-pole (device or simulation).
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    '''

    def __init__(self, config: Config):
        self.config = config

    def reset(self, state: State = State.home()) -> None:
        '''
        Resets the device to the state.
        It must be called at the beginning of any session.
        For real device only position may be set.
        '''
        raise NotImplementedError

    def get_state(self) -> State:
        '''
        Returns current device state.
        '''
        raise NotImplementedError

    def get_info(self) -> Any:
        '''
        Returns usefull debug information.
        '''
        raise NotImplementedError

    def set_target(self, target: float) -> None:
        '''
        Set desired target acceleration.
        '''
        raise NotImplementedError

    def advance(self, delta: float) -> None:
        '''
        Advance system by delta seconds (has means only for simulation).
        '''
        pass

    def timestamp(self) -> float:
        '''
        Current time in seconds (float).
        '''
        raise NotImplementedError

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError
