import enum


class StateKeys(enum.Enum):
    CURR_X = 'curr_x'
    TRGT_X = 'trgt_x'
    CURR_V = 'curr_v'
    TRGT_V = 'trgt_v'
    CURR_A = 'curr_a'
    TRGT_A = 'trgt_a'
    POLE_ANG = 'pole_ang'
    POLE_VEL = 'pole_vel'
    TIMESTAMP = 'timestamp'
    ERRCODE = 'errcode'


class ActionType(enum.IntEnum):
    PositionControl = 1
    VelocityControl = 2
    AccelerationControl = 3


ACTION_TYPE_NAMES = {
    ActionType.PositionControl: StateKeys.TRGT_X.value, 
    ActionType.VelocityControl: StateKeys.TRGT_V.value,
    ActionType.AccelerationControl: StateKeys.TRGT_A.value,
}


class Action:
    def __init__(self, value: float) -> None:
        self.value = value

    def __str__(self) -> str:
        return f'{ACTION_TYPE_NAMES[ActionType.PositionControl]}={self.value}'


class Error(enum.IntEnum):
    NO_ERROR = 0
    NOT_INITIALIZED = 1
    INVALID_CART_POSITION = 2
    INVALID_CART_VELOCITY = 3
    INVALID_CART_ACCELERATION = 4
    TMC_STALL = 5

    def __bool__(self):
        '''
        Returns:
            True if there is any error.
        '''
        return self.value != Error.NO_ERROR


class State:
    """
    Full state of cart-pole device.

    - cart_positon (m):
        Position of the cart along the guide axis.
        The middle is a reference point.

    - cart_velocity (m/s):
        Instantaneous linear speed of the cart.

    - pole_angle (rad):
        Angle of pole (ccw). The he lowest position is the reference point. 

    - pole_velocity (rad/s):
        Instantaneous angular velocity of the pole.

    - error (enum):
        Error codes describing the state of the cart-pole.
    """

    def __init__(self,
            cart_position,
            cart_velocity,
            pole_angle,
            pole_velocity,
            error=Error.NO_ERROR):
        self.cart_position = cart_position
        self.cart_velocity = cart_velocity

        self.pole_angle = pole_angle
        self.pole_velocity = pole_velocity

        self.error = error

    def __bool__(self):
        '''
        Returns:
            True if there is no any error
        '''
        return not self.error

    def __repr__(self):
        return f'(cart_position={self.cart_position}, ' + \
            f'cart_velocity={self.cart_velocity}, ' + \
            f'pole_angle={self.pole_angle}, ' + \
            f'pole_velocity={self.pole_velocity}, ' + \
            f'error={self.error})'
