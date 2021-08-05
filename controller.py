print(__name__)
import typing

import logging

from cart_pole import util
from cart_pole.interface import Error, State, StateKeys, Action
from cart_pole.serial_connection import SerialConnection


LOGGER = logging.getLogger(__name__)


class CartPoleController:
    def __init__(self, serial_connection: SerialConnection, max_steps: int) -> None:
        self.serial = serial_connection
        self.max_steps = max_steps
        self.step_count = 0

    @staticmethod
    def _parse_state(line: str) -> State:
        kv = dict(kv_token.split('=') for kv_token in line.split())
        return State(
            cart_position=float(kv[StateKeys.CURR_X.value]),
            cart_velocity=float(kv[StateKeys.CURR_V.value]),
            pole_angle=float(kv[StateKeys.POLE_ANG.value]),
            pole_velocity=float(kv[StateKeys.POLE_VEL.value]),
            error=Error(int(kv[StateKeys.ERRCODE.value]))
        )

    def state(self) -> State:
        response = self.serial.request(f'state get {StateKeys.CURR_X.value} {StateKeys.CURR_V.value} {StateKeys.CURR_A.value} '
            f'{StateKeys.POLE_ANG.value} {StateKeys.POLE_VEL.value} {StateKeys.TIMESTAMP.value} {StateKeys.ERRCODE.value}')
        return self._parse_state(response)

    def reset(self) -> State:
        """
        Resets the environment. 
        Caution: May take up to 30 seconds to complete.

        Returns:
            Initial State.
        """
        response = self.serial.request('state reset')
        return self._parse_state(response)

    def info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def step(self, action: Action) -> typing.Tuple[State, float, bool, typing.Any]:
        """
        Caution: Currently there is no way to tell precisely when given action will be applied to physical device.

        Returns:
            - State after action has been applied.
            - Reward for given action.
            - Flag that says if environment is finished and needs to be reset before sending new actions.
            - Additional information :see `info` method:
        """

        LOGGER.info(f'Stepping with action {action}')
        _ = self.serial.request(f'state set {action}')

        state = self.state()  # ? we don't know how much time has passed since `state set`

        if not state:
            LOGGER.error(f'Failed to get correct state, got: {state}')
            return state, 0.0, True, self.info()

        self.step_count += 1
        finish = self.step_count >= self.max_steps

        return state, util.reward(state), finish, self.info()
