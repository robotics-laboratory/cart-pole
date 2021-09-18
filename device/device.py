import logging

import util  # FIXME
from interface import (  # FIXME
    Config,
    State,
    CartPoleBase,
)
from device.wire_interface import (  # FIXME
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
        self.target_key = target_key  # FIXME

    def reset(self, config: Config) -> None:
        config.__class__ = DeviceConfig
        self.interface.reset()
        self.interface.set(config)
        self.step_count = 0

    def get_state(self) -> State:
        return self.interface.get(DeviceState.full())

    def get_info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def get_target(self) -> float:
        res = self.interface.get(DeviceTarget(**{self.target_key: True}))
        return getattr(res, self.target_key)

    def set_target(self, target: float) -> None:
        _ = self.interface.set(DeviceTarget(**{self.target_key: target}))

    def close(self) -> None:
        self.interface.close()
