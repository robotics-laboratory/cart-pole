import enum
import numpy
import torch

from pydantic import BaseModel, Field
from typing import Any

import json
import yaml


class Limits(BaseModel):
    '''
    Constraints of cart state, used for control and cartpole parametrization.

    Fields:
        cart_position: max absolute cart position (m)
        cart_velocity: max absolute cart velocity (m/s)
        cart_acceleration: max absolute cart acceleration (m/s^2)
    '''

    cart_position: float = 0.0
    cart_velocity: float = 0.0
    cart_acceleration: float = 0.0

    def stronger(self, other: 'Limits') -> bool:
        '''
        Check if this limit is stronger (more restrictive) than the other.
        '''

        return self.cart_position <= other.cart_position \
            and self.cart_velocity <= other.cart_velocity \
            and self.cart_acceleration <= other.cart_acceleration


class Parameters(BaseModel):
    '''
    CartPole dynamics parameters.Parameters are defined and matter only for simulation.

    Fields:
        g: gravity constant (m/s^2)
        b, k: inner system parameters

    For more information see:
      https://cartpole.robotics-lab.ru/3.0.0/dynamics-and-control
    '''

    g: float = 9.81
    b: float | None = None
    k: float | None = None


class Config(BaseModel):
    '''
    CartPole configuration.

    Fields:
        hardware_limits: hardware limits
        control_limits: control limits
        parameters: dynamics parameters (defined only for simulation)
    '''

    hardware_limit: Limits = Limits()
    control_limit: Limits = Limits()
    parameters: Parameters = Parameters()

    def to_json(self) -> str:
        '''
        Convert config to json string.
        '''

        return self.json(indent=2)
    
    def to_json_file(self, file_path: str) -> None:
        '''
        Save config to json file.
        '''

        with open(file_path, 'w') as f:
            f.write(self.to_json())
    
    @staticmethod
    def from_json(json_str: str) -> 'Config':
        '''
        Load config from json string.
        '''

        return Config.parse_obj(json.loads(json))
    
    @staticmethod
    def from_json_file(file_path: str) -> 'Config':
        '''
        Load config from json file.
        '''

        with open(file_path, 'r') as f:
            return Config.from_json(f.read())
    
    def to_yaml(self) -> str:
        '''
        Convert config to yaml string.
        '''

        return yaml.dump(self.dict(), indent=2)
    
    def to_yaml_file(self, file_path: str) -> None:
        '''
        Save config to yaml file.
        '''

        with open(file_path, 'w') as f:
            f.write(self.to_yaml())

    @staticmethod
    def from_yaml(yaml_str: str) -> 'Cofig':
        '''
        Load config from yaml string.
        '''

        return Config.parse_obj(yaml.load(yaml_str, Loader=yaml.FullLoader))

    @staticmethod
    def from_yaml_file(file_path: str) -> 'Config':
        '''
        Load config from yaml file.

        '''

        with open(file_path, 'r') as f:
            return Config.from_yaml(f.read())


class Error(enum.IntEnum):
    '''
    CartPole system error codes:
        NO_ERROR: no error
        NEED_RESET: reset system before use (make homing)
        CART_POSITION_OVERFLOW: cart position is out of control limit
        CART_VELOCITY_OVERFLOW: cart velocity is out of control limit
        CART_ACCELERATION_OVERFLOW: cart acceleration is out of control limit
        HARDWARE: some hardware error (specific for real device)
    '''

    NO_ERROR = 0
    NEED_RESET = 1
    CART_POSITION_OVERFLOW = 2
    CART_VELOCITY_OVERFLOW = 3
    CART_ACCELERATION_OVERFLOW = 4
    HARDWARE = 5

    def __bool__(self) -> bool:
        return self != Error.NO_ERROR

    def __repr__(self) -> str:
        return str(self.value)

class State(BaseModel):
    '''
    System state:
        cart_position - cart position (m)
        cart_velocity - cart velocity (m/s)
        cart_acceleration - cart acceleration (m/s^2)

        pole_angle - absolute accumulated pole angle (rad)
        pole_angular_velocity - pole angular velocity (rad/s)

        stamp - system time stamp (s)
        error - system error code
    '''

    cart_position: float = 0.0
    cart_velocity: float = 0.0
    cart_acceleration: float = 0.0

    pole_angle: float = 0.0
    pole_angular_velocity: float = 0.0

    stamp: float = 0.0
    error: Error = Error.NO_ERROR

    class Config:
        @staticmethod
        def json_schema_extra(schema: Any, model: Any) -> None:
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
        '''
        Validates state against limits.
        '''

        if self.error:
            return # keep error

        if abs(self.cart_position) > config.control_limit.cart_position:
            self.error = Error.CART_POSITION_OVERFLOW
            return

        if abs(self.cart_velocity) > config.control_limit.cart_velocity:
            self.error = Error.CART_VELOCITY_OVERFLOW
            return
        
        if abs(self.cart_acceleration) > config.control_limit.cart_acceleration:
            self.error = Error.CART_ACCELERATION_OVERFLOW
            return

    def copy(self) -> 'State':
        '''
        Returns deep copy of the state.
        '''
        return State(**self.dict())

    def as_tuple(self) -> tuple[float, float, float, float]:
        '''
        Returns tuple (cart_position, pole_angle, cart_velocity, pole_angular_velocity).
        '''
        return (self.cart_position, self.pole_angle, self.cart_velocity, self.pole_angular_velocity)

    def torch4(self) -> torch.Tensor:
        '''
        Returns state tuple as torch vector.
        '''
        return torch.tensor(self.as_tuple(), dtype=torch.float32)

    def numpy4(self) -> numpy.ndarray:
        '''
        Returns state tuple as numpy vector.
        '''
        return numpy.array(self.as_tuple(), dtype=numpy.float32)


class Target(BaseModel):
    '''
    Control command, used to set desired cart acceleration.

    1. Only acceleration is specified, cart moves with target acceleration.

    2. Velocity is specified, but position is not. Cart reaches velocity, with target acceleration (absolute value needed).
    If acceleration is not specified, used control limit as a default.

    3. Position is specified. Cart reaches position with target velocity/accleration, using bang-bang strategy.
    If velocity/accleration is not specified (absolute value needed), use control limit as a default.
    '''

    position: float | None = None
    velocity: float | None = None
    acceleration: float | None = None

    def acceleration_or(self, default: float) -> float:
        '''
        Returns acceleration or default value.
        '''
        return self.acceleration if self.acceleration is not None else default
    
    def velocity_or(self, default: float) -> float:
        '''
        Returns velocity or default value.
        '''
        return self.velocity if self.velocity is not None else default
    
    def validate(self, config: Config) -> None:
        '''
        Validates target against limits.
        '''

        if self.acceleration is not None:
            assert abs(self.acceleration) <= config.control_limit.cart_acceleration

        if self.velocity is not None:
            assert abs(self.velocity) <= config.control_limit.cart_velocity

            if self.acceleration is not None:
                assert self.acceleration >= 0

        if self.position is not None:
            assert abs(self.position) <= config.control_limit.cart_position

            if self.velocity is not None:
                assert self.velocity >= 0
            if self.acceleration is not None:
                assert self.acceleration >= 0


class CartPoleBase:
    '''
    Description:
      The class specifies a interface of the CartPole (device or simulation).
      A pole is attached by an joint to a cart, which moves along guide axis.
      The pendulum is initially at rest state. The goal is to maintain it in
      upright pose by increasing and reducing cart's acceleration.

    Source:
      This environment is some variation of the cart-pole problem
      described by Barto, Sutton, and Anderson

    Initial state:
      A pole is at starting position 0 with no velocity and acceleration.
    '''

    def get_config(self) -> Config:
        '''
        Returns current configuration, used to identify hardware limits.
        '''
        raise NotImplementedError
    
    def set_config(self, config: Config) -> None:
        '''
        Sets new configuration for the device. Real device ignore hardware limits and parameters.
        '''
        raise NotImplementedError

    def reset(self, state: State = State()) -> None:
        '''
        Resets the device to the state. It must be called at the beginning of any session.
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
        Returns usefull debug information in dict-like object.
        '''
        raise NotImplementedError

    def set_target(self, target: Target) -> State:
        '''
        Set desired target acceleration and returns current state.
        '''
        raise NotImplementedError

    def advance(self, delta: float) -> None:
        '''
        Advance system by delta seconds (has means only for simulation).
        '''
        raise NotImplementedError

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError
