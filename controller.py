import typing

import logging

from cart_pole import util
from cart_pole.interface import State, action, StateInterface
from cart_pole.serial_connection import SerialConnection


LOGGER = logging.getLogger(__name__)


class CartPoleController:
    def __init__(self, serial_connection: SerialConnection, max_steps: int) -> None:
        self.state_interface = StateInterface(serial_connection)
        self.max_steps = max_steps
        self.step_count = 0

    def state(self) -> State:
        request = State(cart_position=1.0, cart_velocity=1.0, pole_velocity=1.0, pole_angle=1.0, error_code=1)
        return self.state_interface.get(request)

    def reset(self) -> State:
        """
        Resets the environment. 
        Caution: May take up to 30 seconds to complete.

        Returns:
            Initial State.
        """
        state = self.state_interface.reset()
        self.step_count = 0
        return state

    def info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def step(self, action_value: float) -> typing.Tuple[State, float, bool, typing.Any]:
        """
        Caution: Currently there is no way to tell precisely when given action will be applied to physical device.

        Returns:
            - State after action has been applied.
            - Reward for given action.
            - Flag that says if environment is finished and needs to be reset before sending new actions.
            - Additional information :see `info` method:
        """

        LOGGER.info(f'Stepping with action {action(action_value).to_dict_format()}')
        _ = self.state_interface.set(action(action_value))
        
        state = self.state()  # ? we don't know how much time has passed since `state set`

        if not state:
            LOGGER.error(f'Failed to get correct state, got: {state}')
            return state, 0.0, True, self.info()

        self.step_count += 1
        finish = self.step_count >= self.max_steps

        return state, util.reward(state), finish, self.info()
