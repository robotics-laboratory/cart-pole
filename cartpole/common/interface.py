import dataclasses
import enum
import math
import numpy
import torch


class Error(enum.IntEnum):
    # CartPole has no error. Default state.
    NO_ERROR = 0

    # Before each session, the device must be reset.
    NEED_RESET = 1

    # Cart has reached its limits or endstop is triggered.
    CART_POSITION_OVERFLOW = 2

    # Cart has reached software velocity limits.
    CART_VELOCITY_OVERFLOW = 3

    # Invalid acceleration target passed.
    CART_ACCELERATION_OVERFLOW = 4

    # Some hardware error occurred. See info for real device.
    HARDWARE = 5

    def __bool__(self) -> bool:
        '''
        Returns True if there is no error.
        '''
        return self != Error.NO_ERROR

@dataclasses.dataclass
class Config:
    '''
    CartPole configuration, which are essential for control algorithms.
    It is used to initialize the device before session.
    Config may be extended for specific device.
    '''

    # software cart limits
    max_position: float = 0.25      # m
    max_velocity: float = 2.0       # m/s
    max_acceleration: float = 3.5   # m/s^2

    # software pole limits
    max_pole_angle = 3 * 2 * PI
    max_pole_velocity = 3 * 2 * PI

@dataclasses.dataclass
class State:
    '''
    State of CartPole system. All essential variables for control are stored here.
    '''

    cart_position: float = 0         # m
    cart_velocity: float = 0         # m/s
    pole_angle: float = 0            # rad
    pole_angular_velocity: float = 0 # rad/s
    error: Error = Error.NO_ERROR
 
    # Some specific device may store additional info here
    # for example simulation step counter, accelerometer readings, voltage, etc.
    debug_info: dict[str, Any] = dict()

    @staticmethod
    def from_numpy(q):
        '''
        q = (x, a, v, w)
        '''
        return State(a[0], a[2], a[1], a[3])

    @staticmethod
    def from_torch(q):
        '''
        q = (x, a, v, w)
        '''
        return State(q[0].item(), q[2].item(), q[1].item(), q[3].item())

    @staticmethod
    def home():
        return State(.0, .0, .0, .0)

    def as_tuple(self):
        return (
            self.cart_position,
            self.pole_angle,
            self.cart_velocity,
            self.pole_angular_velocity,
        )

    def as_numpy(self):
        return np.array(self.as_tuple()).reshape(4, 1)

    def as_torch(self):
        return torch.torch(self.as_array()).reshape(4, 1)

    def __repr__(self):
        return '(x={x:+.2f}, v={v:+.2f}, a={a:+.2f}, w={w:+.2f}, err={err})'.format(
            x = self.cart_position,
            v = self.cart_velocity,
            a = self.pole_angle,
            w = self.pole_angular_velocity,
            err=self.error,
        )

class CartPoleBase:
    '''
    Description:
        Сlass implements a base cart-pole device.
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing the cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    '''

    def reset(self, config: Config) -> State:
        '''
        Resets the device to the initial state.
        The pole is at rest position and cart is centered.
        It must be called at the beginning of any session.
        '''
        raise NotImplementedError()

    def set_target(self) ->State:
        '''
        Sets the target acceleration and returns current state.
        '''
        raise NotImplementedError()

    def get_target(self) -> State:
        '''
        Returns current target acceleration.
        '''
        raise NotImplementedError()

    def get_state(self) -> State:
        '''
        Returns current device state.
        '''
        raise NotImplementedError()

    def advance(self, delta: float = None) -> None:
        '''
        Advance the dynamic system by delta seconds (has meaning only for simulation).
        '''
        raise NotImplementedError()

    def timestamp(self) -> float:
        '''
        Current time in seconds.
        '''
        raise NotImplementedError()

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError()
