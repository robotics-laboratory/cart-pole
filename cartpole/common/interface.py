import enum
import numpy
import torch

from pydantic import BaseModel, Field
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

    def __str__(self) -> str:
        return self.name

    def __repr__(self) -> str:
        return str(self.value)


class Config(BaseModel):
    # software cart limits
    max_cart_position: float = 0.25  # m
    max_cart_velocity: float = 2.0  # m/s
    max_cart_acceleration: float = 3.5  # m/s^2

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

    class Config:
        @staticmethod
        def schema_extra(schema: Any, model: Any) -> None:
            # make schema lightweight
            schema.pop('definitions', None)

            properties = schema['properties']           
            for name in properties:
                properties[name].pop('title', None)

            # simplify schema for foxglove
            properties['error'] = {
                'type': 'integer',
                'enum': [e.value for e in Error]
            }

    def validate(self, config: Config) -> None:
        if self.error:
            return

        if abs(self.cart_position) > config.max_cart_position:
            self.error = Error.CART_POSITION_OVERFLOW
            return

        if abs(self.cart_velocity) > config.max_cart_velocity:
            error = Error.CART_VELOCITY_OVERFLOW
            return
        
        if abs(self.cart_acceleration) > config.max_cart_acceleration:
            error = Error.CART_ACCELERATION_OVERFLOW
            return

#    @staticmethod
#    def home() -> State:
#        return State()

    def as_tuple(self):
        '''
        Returns state (cart_position, cart_velocity, pole_angle, pole_angular_velocity) as tuple.
        '''
        return (self.cart_position, self.pole_angle, self.cart_velocity, self.pole_angular_velocity)

    def torch4(self):
        '''
        Returns state tuple as torch tensor.
        '''
        return torch.tensor(self.as_tuple(), dtype=torch.float32)

    def numpy4(self):
        '''
        Returns state tuple as numpy array.
        '''
        return numpy.array(self.as_tuple(), dtype=numpy.float32)

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

    def reset(self, state: State = State()) -> None:
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
