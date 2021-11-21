import enum

import numpy as np

class Error(enum.IntEnum):
    NO_ERROR = 0
    NEED_RESET = 1
    X_OVERFLOW = 2
    V_OVERFLOW = 3
    A_OVERFLOW = 4
    MOTOR_STALLED = 5
    ENDSTOP_HIT = 6

    def __bool__(self) -> bool:
        return self != Error.NO_ERROR

class Config:
    def __init__(self,
            # software cart limits
            max_position=0.25,    # m
            max_velocity=2.0,     # m/s
            max_acceleration=3.5, # m/s^2
            # hardware limits
            hard_max_position=0.27,    # m
            hard_max_velocity=2.5,     # m/s
            hard_max_acceleration=5.0, # m/s^2
            # physical params
            pole_length=0.3, # m
            pole_mass=0.118, # kg
            gravity=9.8):    # m/s^2
        
        self.max_position = max_position
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        self.hard_max_position = hard_max_position
        self.hard_max_velocity = hard_max_velocity
        self.hard_max_acceleration = hard_max_acceleration

        self.pole_length = pole_length
        self.gravity = gravity

    def __repr__(self):
        return str(vars(self))

class State:
    def __init__(self,
            cart_position,
            cart_velocity,
            pole_angle,
            pole_angular_velocity,
            error=Error.NO_ERROR):
        self.cart_position = cart_position
        self.cart_velocity = cart_velocity
        self.pole_angle = pole_angle
        self.pole_angular_velocity = pole_angular_velocity
        self.error = error
    
    @staticmethod
    def from_array(a):
        '''
        q = (x, a, v, w)
        '''
        return State(a[0], a[2], a[1], a[3])

    def bool(self):
        return not self.error_code

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

    def as_array(self):
        return np.array(self.as_tuple())

    def as_array_4x1(self):
        return self.as_array().reshape(4, 1)

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
        Сlass implements a physical simulation of the cart-pole device.
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing the cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    '''

    def reset(self, config: Config) -> None:
        '''
        Resets the device to the initial state.
        The pole is at rest position and cart is centered.
        It must be called at the beginning of any session.
        '''
        raise NotImplementedError

    def get_state(self) -> State:
        '''
        Returns current device state.
        '''
        raise NotImplementedError

    def get_info(self) -> dict:
        '''
        Returns usefull debug information.
        '''
        raise NotImplementedError

    def get_target(self) -> float:
        '''
        Returns current target acceleration.
        '''
        raise NotImplementedError

    def set_target(self, target: float) -> None:
        '''
        Set desired target acceleration.
        '''
        raise NotImplementedError

    def advance(self, delta) -> None:
        '''
        Advance the dynamic system by delta seconds.
        '''
        pass

    def timestamp(self):
        '''
        Current time.
        '''
        raise NotImplementedError

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError
