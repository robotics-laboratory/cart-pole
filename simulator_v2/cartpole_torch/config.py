"""
Defines configuration of the system - its limits and physical parameters
"""
from dataclasses import dataclass

from torch import pi


@dataclass
class SystemLimits:
    """
    Represents system limits

    Fields
    ------
    `max_abs_position` : float
        Maximum absolute position of the cart, so the real
        position belongs to (-max_abs_position, max_abs_position).
        Measured in meters.

    `max_abs_velocity` : float
        Maximum absolute velocity of the cart.
        Measured in m/s (meters per second).

    `max_abs_acceleration` : float
        Maximum absolute acceleration of the cart.
        Measured in m/s^2 (meters per second squared).
    """

    max_abs_position: float = 0.25
    max_abs_velocity: float = 25.0
    max_abs_angular_velocity: float = 6.0 * pi
    max_abs_acceleration: float = 7


@dataclass
class DiscretizationParameters:
    """
    This class contains the parameters used for discretization
    of state space and time.

    Each field except `input_time` defines the number of samples
    for each dimension.
    `cart_position = 50` would mean that we take 50 equidistant points
    from `[-max_abs_position, max_abs_position]`.

    `input_time` stores the discretization of input times.
    Value of `100` would mean that we can change out input 100 times a second.

    `dynamics_per_time_step` is the number
    """

    cart_position: int = 50
    pole_angle: int = 21
    cart_velocity: int = 50
    pole_angular_velocity: int = 126
    cart_acceleration: int = 35

    input_time: int = 100


@dataclass
class SystemParameters:
    """
    Represents physical parameters of the system

    Fields
    ------
    `pole_length` : float
        Length of the pole in meters.

    `pole_mass` : float
        Mass of the pole in kg.
    
    `cart_mass` : float
        Mass of the cart in kg.

    `gravity` : float
        Gravitational constant, shows the gravitational pull.
        Measured in m/s^2 (meters per second squared).
    """

    pole_length: float = 0.2
    pole_mass: float = 0.118
    cart_mass: float = 0
    gravity: float = 9.807


@dataclass
class SystemConfiguration:
    """
    Contains physical parameters of the system and its limits.

    Fields
    ------
    `parameters` : SystemParameters
        Physical parameters of the system.

    `limits` : SystemLimits
        Limits of the system, such as maximum position, etc.

    `discretization` : DiscretizationParameters
        Parameters of the system which are used for discretization of
        continuos space and time.
        Basically contains the number of pieces we split each dimension in.

    `dynamics_steps_per_input` : int
        Shows how many times we calculate dynamics of the system during
        one timestep.
        We assume the input is the same during the timestep.
        Value of 10 would mean that we update the state of the system
        10 times before adjusting the input.
    """

    parameters: SystemParameters = SystemParameters()
    limits: SystemLimits = SystemLimits()
    discretization: DiscretizationParameters = DiscretizationParameters()
    dynamics_steps_per_input: int = 10
