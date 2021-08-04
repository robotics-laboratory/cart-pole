import typing

import serial
import enum
import logging

import util
from state import State, Error


LOGGER = logging.getLogger('controller')

class StateKeys:
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


class ActionType(enum.Enum):
    PositionControl = 1
    VelocityControl = 2
    AccelerationControl = 3


class Action:
    def __init__(self, action_type: ActionType, value: float) -> None:
        self.type = action_type
        self.value = value

    def __str__(self) -> str:
        action_str = {
            ActionType.PositionControl: StateKeys.TRGT_X, 
            ActionType.VelocityControl: StateKeys.TRGT_V,
            ActionType.AccelerationControl: StateKeys.TRGT_A,
            }[self.type]
        return f'{action_str}={self.value}'


class CartPoleController:
    def __init__(self, device: str, baud_rate: int, step_n: int) -> None:
        self.serial = serial.Serial(port=device, baudrate=baud_rate)
        LOGGER.info(f'Opened serial connection to port {self.serial.name}')

        self.step_n = step_n
        self.step_count = 0

    @staticmethod
    def _parse_state(line: str) -> State:
        kv = dict(kv_token.split('=') for kv_token in line.split())
        return State(
            cart_position=float(kv[StateKeys.CURR_X]),
            cart_velocity=float(kv[StateKeys.CURR_V]),
            pole_angle=float(kv[StateKeys.POLE_ANG]),
            pole_velocity=float(kv[StateKeys.POLE_VEL]),
            error=Error(int(kv[StateKeys.ERRCODE]))
        )

    @staticmethod
    def _log_get_error(line: str):
        if line.startswith('!'):
            LOGGER.error(f'Failed to get state, got response: {line}')

    def state(self) -> State:
        self.serial.write(f'state get {StateKeys.CURR_X} {StateKeys.CURR_V} {StateKeys.CURR_A} '
            f'{StateKeys.POLE_ANG} {StateKeys.POLE_VEL} {StateKeys.TIMESTAMP} {StateKeys.ERRCODE}')
        line = self.serial.readline().lower()
        return self._parse_state(line)

    def reset(self) -> State:
        self.serial.write('state reset')
        line = self.serial.readline()
        return self._parse_state(line)

    def info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def step(self, action: Action) -> typing.Tuple[State, float, bool, typing.Any]:
        self.serial.write(f'state set {action}')
        line: str = self.serial.readline()
        if line.startswith('!'):
            LOGGER.error(f'Failed to set state with action {action}, got response: {line}')
            return None, 0.0, True, self.info()

        state = self.state()  # ? we don't know how much time has passed since `state set`

        if not state:
            LOGGER.error(f'Failed to get correct state, got: {state}')
            return state, 0.0, True, self.info()

        self.step_count += 1
        if self.step_count == self.step_n:
            finish = True
        else:
            finish = False

        return state, util.reward(state), finish, self.info()
