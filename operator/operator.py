import logging
import time
from typing import Tuple

from cart_pole.common import util
from cart_pole.common.interface import (
    Config,
    State,
    Target,
    CartPoleBase,
)
from cart_pole.operator.wire_interface import WireInterface


LOGGER = logging.getLogger(__name__)


class Operator(CartPoleBase):
    def __init__(self, interface: WireInterface = None) -> None:
        self.interface = interface or WireInterface()
        self.step_count = 0
        self.get_state_timestamp = None

    def reset(self, config: Config) -> Tuple[State, float]:
        self.interface.reset()
        self.interface.set(config)
        state: State = self.interface.get(State.full())
        target = self.interface.get(Target(position=True)).position
        self.step_count = 0
        self.get_state_timestamp = None
        return state, target

    def step(self, delta: float) -> None:
        assert self.get_state_timestamp is not None, '`get_state` was never called'
        actual_delta = max(0, time.time() - self.get_state_timestamp)
        if actual_delta > delta:
            LOGGER.debug('Actual time delta is greater that requested')
        else:
            LOGGER.debug(f'Actual time delta is less that requested, sleeping for {delta - actual_delta} seconds')
            time.sleep(delta - actual_delta)

    def get_state(self) -> State:
        self.get_state_timestamp = time.time()
        return self.interface.get(State.full())

    def get_info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def get_target(self) -> float:
        return self.interface.get(Target(position=True)).position

    def set_target(self, target: float) -> None:
        _ = self.interface.set(Target(position=target))
    
    def close(self) -> None:
        self.interface.close()
