import logging

from common import util
from common.interface import (
    Config,
    State,
    CartPoleBase,
)
from device.wire_interface import (
    WireInterface,
    DeviceTarget,
    DeviceConfig,
    DeviceState,
)


LOGGER = logging.getLogger(__name__)


class CartPoleDevice(CartPoleBase):
    def __init__(self, interface: WireInterface = None, target_key='acceleration') -> None:
        self.interface = interface or WireInterface()
        self.step_count = 0
        self.target_key = target_key

    def reset(self, config: Config = None) -> None:
        self.interface.reset()
        config = config or Config.default()
        config.__class__ = DeviceConfig
        self.interface.set(config)
        self.step_count = 0

    def get_state(self) -> State:
        return self.interface.get(DeviceState.full())

    def get_info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def get_target(self) -> float:
        request = DeviceTarget(**{self.target_key: True})
        target = self.interface.get(request)
        return getattr(target, self.target_key)

    def set_target(self, target: float) -> None:
        request = DeviceTarget(**{self.target_key: target})
        self.interface.set(request)

    def close(self) -> None:
        self.interface.close()
