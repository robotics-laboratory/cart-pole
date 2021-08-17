import dataclasses as dc
import enum
from typing import Type, Tuple


class Error(enum.Enum):
    NO_ERROR = 0
    NEED_RESET = 1
    X_OVERFLOW = 2
    V_OVERFLOW = 3
    A_OVERFLOW = 4
    MOTOR_STALLED = 5
    ENDSTOP_HIT = 6

    def __bool__(self) -> bool:
        '''
        Returns:
            True if there is any error.
        '''
        return self.value != Error.NO_ERROR


WIRE_NAME = 'wire_name'


class DeviceVariableGroup:
    '''
    Mixin intended to be used with @dataclass-decorated classes. Subclass should
    represent a variable group in a controller protocol spec. Group name should be
    set by overriding `PROTOCOL_GROUP_NAME` classvar. Mixin provides methods
    for convertion to and from wire formats.
    '''

    # Should be overridden in subclass
    PROTOCOL_GROUP_NAME: str = None

    # Field formatters (value -> string)
    _to_string_lookup = {
        float: lambda v: f'{round(v, 5)}',
        bool: lambda v: 'true' if v else 'false',
        int: lambda v: str(v),
        Error: lambda v: str(v.value),
    }

    # Field parsers (string -> value)
    _from_string_lookup = {
        float: lambda s: float(s),
        bool: lambda s: s == 'true',
        int: lambda s: int(s),
        Error: lambda s: Error(int(s)),
    }

    def __init_subclass__(cls) -> None:
        super().__init_subclass__()
        assert cls.PROTOCOL_GROUP_NAME is not None, 'PROTOCOL_GROUP_NAME is not set'

    @classmethod
    def _type_lookup(cls, type: Type, kv: dict):
        for possible_type, value in kv.items():
            if issubclass(type, possible_type):
                return value
        raise ValueError(f'No matching item for type {type}')

    @classmethod
    def full(cls) -> 'DeviceVariableGroup':
        '''
        Constructs instance initialized with filler value
        '''
        return cls(**{field.name: True for field in dc.fields(cls)})

    def to_dict_format(self) -> str:
        '''
        Serializes current dataclass instance to '<key>=<value>' format
        as used in 'set' method of the protocol.

        Returns:
            Command string in dict wire format.
        '''
        return ' '.join(
            f'{field.metadata[WIRE_NAME]}={self._type_lookup(field.type, self._to_string_lookup)(getattr(self, field.name))}'
            for field in dc.fields(self)
            if getattr(self, field.name) is not None
        )

    def to_list_format(self) -> str:
        '''
        Serializes current dataclass instance to '<key>' format
        as used in 'get' method of the protocol.

        Returns:
            Command string in list wire format.
        '''
        return ' '.join(field.metadata[WIRE_NAME] for field in dc.fields(self) if getattr(self, field.name) is not None)

    @classmethod
    def from_dict_format(cls, text: str) -> 'DeviceVariableGroup':
        '''
        Parses input text into new class instance.

        Returns:
            Class instance, constructed from wire format representation.
        '''
        field_lookup = {field.metadata[WIRE_NAME]: field for field in dc.fields(cls)}
        data_dict = {}
        for pair in text.split():
            wire_name, value = pair.split('=')
            field = field_lookup[wire_name]
            from_string = cls._type_lookup(field.type, cls._from_string_lookup)
            data_dict[field.name] = from_string(value)
        return cls(**data_dict)


def make_field(wire_name: str):
    '''
    Makes field descriptor that maps to a different name in controller protocol.
    :param wire_name: field name as defined in conrtoller protocol spec.
    :return: dataclasses.Field instance
    '''
    return dc.field(default=None, metadata={WIRE_NAME: wire_name})


@dc.dataclass
class Config(DeviceVariableGroup):
    PROTOCOL_GROUP_NAME = 'config'

    max_position: float = make_field('max_x')
    max_velocity: float = make_field('max_v')
    max_acceleration: float = make_field('max_a')
    hardware_max_postition: float = make_field('hw_max_x')
    hardware_max_velocity: float = make_field('hw_max_v')
    hardware_max_acceleration: float = make_field('hw_max_a')
    clamp_position: bool = make_field('clamp_x')
    clamp_velocity: bool = make_field('clamp_v')
    clamp_acceleration: bool = make_field('clamp_a')


@dc.dataclass
class State(DeviceVariableGroup):
    PROTOCOL_GROUP_NAME = 'state'

    position: float = make_field('x')
    velocity: float = make_field('v')
    acceleration: float = make_field('a')
    pole_angle: float = make_field('pole_x')
    pole_angular_velocity: float = make_field('pole_v')
    error_code: Error = make_field('errcode')


@dc.dataclass
class Target(DeviceVariableGroup):
    PROTOCOL_GROUP_NAME = 'target'

    position: float = make_field('x')
    velocity: float = make_field('v')
    acceleration: float = make_field('a')


class CartPoleBase:
    def reset(self, config: Config) -> Tuple[State, float]:
        '''
        Resets the device to the initial state.
        The pole is at rest position and cart is centered.
        It must be called at the beginning of any session.
        Returns (initial_state, initial_target).
        '''
        raise NotImplementedError

    def step(self, delta: float) -> None:
        '''
        Ensures that `delta` seconds has passed since 
        previous `get_state` call.
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
        Returns current target value.
        '''
        raise NotImplementedError

    def set_target(self, target: float) -> None:
        '''
        Set desired target value.
        '''
        raise NotImplementedError

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError
