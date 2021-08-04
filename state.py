import enum

class Error(enum.IntEnum):
    NO_ERROR = 0
    NOT_INITIALIZED = 1
    INVALID_CART_POSITION = 2
    INVALID_CART_VELOCITY = 3 # future

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
