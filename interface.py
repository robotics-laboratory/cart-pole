import enum
import inspect
from typing import Union

from cart_pole.serial_connection import SerialConnection
from cart_pole.util import as_bool
    

class Error(enum.IntEnum):
    NO_ERROR = 0
    NOT_INITIALIZED = 1
    INVALID_CART_POSITION = 2
    INVALID_CART_VELOCITY = 3
    INVALID_CART_ACCELERATION = 4
    TMC_STALL = 5

    def __bool__(self) -> bool:
        '''
        Returns:
            True if there is any error.
        '''
        return self.value != Error.NO_ERROR


class MetaParameters(type):
    WIRE_TO_CODE_ATTRIBUTE_NAME = 'WIRE_TO_CODE_NAME_MAP'
    CODE_TO_WIRE_ATTRIBUTE_NAME = 'CODE_TO_WIRE_NAME_MAP'

    def __new__(cls, name, bases, dct):
        if '__init__' in dct:
            assert 'Keys' in dct, 'Class must have `Keys` subclass with wire format names in same order as in `__init__` method'
            keys = dct['Keys']
            assert isinstance(keys, type), 'Must be class'

            wire_format_names = [v for k, v in vars(keys).items() if not k.startswith('_')]

            args, varargs, varkw, defaults, kwonlyargs, kwonlydefaults, annotations = inspect.getfullargspec(dct['__init__'])
            assert args == ['self'], 'Must have no arguments'
            assert varargs is None, 'Must have no variable args'
            assert varkw is None, 'Must have no variable kwargs'
            assert defaults is None  # this follows from `args` assert, but just in case
            assert len(kwonlydefaults) == len(kwonlyargs), 'All kw-only args must have defaults'
            assert all([v is None for v in kwonlydefaults.values()]), 'All kw-only defaults must be `None`s'

            code_format_names = kwonlyargs

            assert len(wire_format_names) == len(code_format_names), 'Number of values in `Keys` subclass must match number of arguments in `__init__` method'
            
            wire_to_code_map = dict((wire, code) for wire, code in zip(wire_format_names, code_format_names))
            code_to_wire_map = dict((code, wire) for wire, code in zip(wire_format_names, code_format_names))

            assert cls.WIRE_TO_CODE_ATTRIBUTE_NAME not in dct, f'Class must have no attribute named `{cls.WIRE_TO_CODE_ATTRIBUTE_NAME}`'
            assert cls.CODE_TO_WIRE_ATTRIBUTE_NAME not in dct, f'Class must have no attribute named `{cls.CODE_TO_WIRE_ATTRIBUTE_NAME}`'

            dct[cls.WIRE_TO_CODE_ATTRIBUTE_NAME] = wire_to_code_map
            dct[cls.CODE_TO_WIRE_ATTRIBUTE_NAME] = code_to_wire_map

        return super().__new__(cls, name, bases, dct)


class Parameters(metaclass=MetaParameters):
    @classmethod
    def _wire_to_code_attribute_map(cls):
        return getattr(cls, cls.__class__.WIRE_TO_CODE_ATTRIBUTE_NAME)

    @classmethod
    def _code_to_wire_attribute_map(cls):
        return getattr(cls, cls.__class__.CODE_TO_WIRE_ATTRIBUTE_NAME)

    @classmethod
    def parse_from_dict_format(cls, line: str) -> 'Parameters':
        kv = [kv_token.split('=') for kv_token in line.split()]
        wtc = cls._wire_to_code_attribute_map()
        return cls(**dict((wtc[k], v) for k, v in kv))

    def __repr__(self) -> str:
        attr_names = self._code_to_wire_attribute_map().keys()
        formatted_attrs = [f'{attr_name}={getattr(self, attr_name)}' for attr_name in attr_names if getattr(self, attr_name)]
        pieces = [attr + ', ' for attr in formatted_attrs[:-1]]
        if len(formatted_attrs):
            pieces.append(formatted_attrs[-1])
        return ''.join([s for s in [self.__class__.__name__, '('] + pieces + [')']])

    def to_list_format(self) -> str:
        wtc = self._wire_to_code_attribute_map()
        return ' '.join([wire_name for wire_name, code_name in wtc.items() if getattr(self, code_name)])

    def to_dict_format(self) -> str:
        wtc = self._wire_to_code_attribute_map()
        return ' '.join([f'{wire_name}={getattr(self, code_name)}' for wire_name, code_name in wtc.items() if getattr(self, code_name)])
            

class State(Parameters):
    """
    Full state of cart-pole device.

    - cart_position (m):
        Position of the cart along the guide axis.
        The middle is a reference point.
    
    - target_position (m):
        Here be dragons

    - cart_velocity (m/s):
        Instantaneous linear speed of the cart.

    - target_velocity (m/s):
        Here be dragons

    - cart_acceleration (m/s^2):
        Instantaneous linear acceleration of the cart.

    - target_acceleration (m/s^2):
        Here be dragons

    - pole_angle (rad):
        Angle of pole (ccw). The lowest position is the reference point. 

    - pole_velocity (rad/s):
        Instantaneous angular velocity of the pole.

    - timestamp
        Here be dragons

    - error_code
        Here be dragons

    - error (enum):
        Error code describing the state of the cart-pole.
    """

    class Keys:
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

    def __init__(self, *,
            cart_position: Union[float, str] = None,
            target_position: Union[float, str] = None,
            cart_velocity: Union[float, str] = None,
            target_velocity: Union[float, str] = None,
            cart_acceleration: Union[float, str] = None,
            target_acceleration: Union[float, str] = None,
            pole_angle: Union[float, str] = None,
            pole_velocity: Union[float, str] = None,
            timestamp: Union[float, str] = None,
            error_code: Union[int, str] = None
        ) -> None:
        self.cart_position = float(cart_position or 0.0)
        self.target_position = float(target_position or 0.0)
        self.cart_velocity = float(cart_velocity or 0.0)
        self.target_velocity = float(target_velocity or 0.0)
        self.cart_acceleration = float(cart_acceleration or 0.0)
        self.target_acceleration = float(target_acceleration or 0.0)
        self.pole_angle = float(pole_angle or 0.0)
        self.pole_velocity = float(pole_velocity or 0.0)
        self.timestamp = float(timestamp or 0.0)
        self.error_code = int(error_code or 0)

        self.error = Error(self.error_code)

    def __bool__(self) -> bool:
        '''
        Returns:
            True if there is no any error
        '''
        return not self.error


def action(value: float) -> State:
    return State(target_position=value)


class Config(Parameters):
    class Keys:
        SAFE_MARGIN = 'safe_margin'
        MAX_X = 'max_x'
        MAX_V = 'max_v'
        MAX_A = 'max_a'
        HW_MAX_V = 'hw_max_a'
        HW_MAX_A = 'hw_max_a'
        CLAMP_X = 'clamp_x'
        CLAMP_V = 'clamp_v'
        CLAMP_A = 'clamp_a'
        STEPS_PER_MM = 'steps_per_mm'
        STEPPER_CURRENT = 'stepper_current'

    def __init__(self, *,
        safe_margin: Union[float, str] = None,
        max_position: Union[float, str] = None,
        max_velocity: Union[float, str] = None,
        max_acceleration: Union[float, str] = None,
        hardware_max_velocity: Union[float, str] = None,
        hardware_max_acceleration: Union[float, str] = None,
        clamp_position: Union[bool, str] = None,
        clamp_velocity: Union[bool, str] = None,
        clamp_acceleration: Union[bool, str] = None,
        steps_per_mm: Union[float, str] = None,
        stepper_current: Union[float, str] = None,
    ) -> None:
        self.safe_margin = float(safe_margin or 0.0)
        self.max_position = float(max_position or 0.0)
        self.max_velocity = float(max_velocity or 0.0)
        self.max_acceleration = float(max_acceleration or 0.0)
        self.hardware_max_velocity = float(hardware_max_velocity or 0.0)
        self.hardware_max_acceleration = float(hardware_max_acceleration or 0.0)
        self.clamp_position = as_bool(clamp_position)
        self.clamp_velocity = as_bool(clamp_velocity)
        self.clamp_acceleration = as_bool(clamp_acceleration)
        self.steps_per_mm = float(steps_per_mm or 0.0)
        self.stepper_current = float(stepper_current or 0.0)


class SerialInterface:
    def __init__(self, serial_connection: SerialConnection, command: str) -> None:
        self.serial_connection = serial_connection
        self.command = command

    def _request(self, method: str, args: str = '') -> str:
        return self.serial_connection.request(f'{self.command} {method} {args}')


class BaseInterface(SerialInterface):
    def __init__(self, serial_connection: SerialConnection, command: str, param_class: type) -> None:
        super().__init__(serial_connection, command)
        self._param_class = param_class

    def set(self, params):
        response = self._request('set', params.to_dict_format())
        return self._param_class.parse_from_dict_format(response)

    def get(self, params):
        response = self._request('get', params.to_list_format())
        return self._param_class.parse_from_dict_format(response)

    def reset(self):
        response = self._request('reset')
        return self._param_class.parse_from_dict_format(response)


class ConfigInterface(BaseInterface):
    def __init__(self, serial_connection: SerialConnection) -> None:
        super().__init__(serial_connection, 'config', Config)


class StateInterface(BaseInterface):
    def __init__(self, serial_connection: SerialConnection) -> None:
        super().__init__(serial_connection, 'state', State)
