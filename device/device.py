import logging

from cart_pole import util
from cart_pole.interface import (
    Config,
    State,
    Target,
    CartPoleBase,
)
from cart_pole.device.wire_interface import WireInterface


LOGGER = logging.getLogger(__name__)


class CartPoleDevice(CartPoleBase):
    def __init__(self, interface: WireInterface = None) -> None:
        self.interface = interface or WireInterface()
        self.step_count = 0

    def reset(self, config: Config) -> None:
        self.interface.reset()
        self.interface.set(config)
        self.step_count = 0

    def get_state(self) -> State:
        return self.interface.get(State.full())

    def get_info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def get_target(self) -> float:
        return self.interface.get(Target(position=True)).position

    def set_target(self, target: float) -> None:
        _ = self.interface.set(Target(position=target))
    
    def close(self) -> None:
        self.interface.close()
