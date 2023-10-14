try:
    from cartpole.device import proto
    from cartpole.device.proto import RequestType
except ImportError as err:
    raise RuntimeError(
        "Failed to import protobuf bindings. "
        "Maybe you forgot to run scripts/compile_protobuf.py?"
    ) from err

import logging
import time
from typing import NamedTuple, Optional, Union

from betterproto import Casing, Message
from pydantic import BaseModel, BaseSettings, FilePath
from serial import Serial
from serial.tools.list_ports import comports

from cartpole.common import CartPoleBase
from cartpole.common.interface import Config, Error, State
from cartpole.device import framing


class ConnectionSettings(BaseSettings):
    """
    Device connection settings. Can be set via environment variables, or overriden
    directly by passing kwargs to CartPoleDevice constructor.

    - serial_port: path to serial device (e.g. /dev/serial/by-id/xxx)
    - serial_speed: serial baud rate (bits/s)
    - serial_timeout: serial read timeout (s)
    - hard_reset: whether to perform hard device reset upon connecting
    """

    serial_port: FilePath = None
    serial_speed: int = 1000000
    serial_timeout: float = 0.1
    hard_reset: bool = True

    @classmethod
    def with_overrides(cls, **kwargs):
        kwargs = {k: v for k, v in kwargs.items() if v is not None}
        return cls(**kwargs)


class TypeMapping(NamedTuple):
    """
    Helper structure to define request/response arguments and mapping between python
    dataclasses (pydantic models) and protobuf (betterproto) messages. Setting some
    argument to None means request/response does not accept (return) any data (e.g.
    reset command).
    """

    req_model: Optional[BaseModel] = None
    req_message: Optional[Message] = None
    res_model: Optional[BaseModel] = None
    res_message: Optional[Message] = None


# TODO: Move to cartpole.common.interface?
class Target(BaseModel):
    position: Optional[float] = None
    velocity: Optional[float] = None
    acceleration: Optional[float] = None


class DeviceConfig(Config, proto.Config):
    pass  # Use protobuf type as-is (for now)


class CartPoleDevice(CartPoleBase):
    REQUEST_MAPPING = {
        # Reset command: accepts nothing, returns nothing
        RequestType.RESET: TypeMapping(),
        # TODO: Custom DeviceState class?
        # Set target command: accepts target, returns state
        RequestType.TARGET: TypeMapping(Target, proto.Target, State, proto.State),
        # Set config command: accepts config, returns config
        RequestType.CONFIG: TypeMapping(
            DeviceConfig, proto.Config, DeviceConfig, proto.Config
        ),
    }
    RESET_DELAY = 0.1  # Delay fr
    HOMING_TIMEOUT = 10

    def __init__(self, **kwargs):
        cfg = ConnectionSettings.with_overrides(**kwargs)
        cfg.serial_port = cfg.serial_port or self._detect_port()
        logging.info(f"Serial: {cfg.serial_port} @ {cfg.serial_speed} bits/s")
        self._port = Serial(
            port=cfg.serial_port,
            baudrate=cfg.serial_speed,
            timeout=cfg.serial_timeout,
            exclusive=True,
        )
        if cfg.hard_reset:
            self._hard_reset()

    def _detect_port(self):
        all_ports = list(comports())
        if len(all_ports) == 0:
            raise RuntimeError("No serial ports detected")
        if len(all_ports) > 1:
            msg = "Too many serial ports, please select one via SERIAL_PORT env var:"
            for port, desc, hwid in all_ports:
                msg += f"\n- {port} : {desc} [{hwid}]"
            raise RuntimeError(msg)
        return all_ports[0][0]

    def _hard_reset(self):
        logging.info("Hard-resetting device...")
        self._port.rts = True
        self._port.dtr = False
        time.sleep(self.RESET_DELAY)
        self._port.rts = False
        time.sleep(self.RESET_DELAY)
        # TODO: Consistent restart marker?
        data = self._port.read_until(b"start")
        self._port.read_all()
        logging.debug(f"Purged data: {data.decode(errors='ignore')}")

    def _request(self, type: RequestType, payload: Union[BaseModel, Message] = None):
        logging.debug(f"SEND: {type!r} request with payload: {payload}")
        mapping = self.REQUEST_MAPPING[type]
        if payload is not None:
            if not isinstance(payload, Message):
                payload = mapping.req_message(**payload.dict())
            assert isinstance(payload, mapping.req_message)
        data = framing.encode(type)
        self._port.write(data)
        logging.debug(f"SEND (raw): {data}")
        data = self._port.read_until(framing.FRAME_DELIMITER)
        logging.debug(f"RECV (raw): {data}")
        if not data:
            logging.error(f"{type!r} request timeout")
            raise TimeoutError
        response_type, response = framing.decode(data, mapping.res_message)
        assert type == response_type, "Unexpected response type"
        if response is not None:
            response = mapping.res_model(**response.to_dict(Casing.SNAKE))
        logging.debug(f"RECV: {response_type} response with payload: {response}")
        return response

    def reset(self, state: State = State()):
        self._request(RequestType.RESET)
        start = time.perf_counter()
        state: State = self._request(RequestType.TARGET, Target())
        while state.error == Error.NO_ERROR:
            state = self._request(RequestType.TARGET, Target())
            if time.perf_counter() - start > self.HOMING_TIMEOUT:
                raise RuntimeError("Device homing timeout")
        if state.cart_position != 0:
            # TODO: Move to position (if specified)
            raise NotImplementedError


if __name__ == "__main__":
    from cartpole.common.util import init_logging

    init_logging()

    iterations = 1000
    start = time.perf_counter()
    device = CartPoleDevice()
    for _ in range(iterations):
        state = device._request(
            RequestType.TARGET, Target(position=1, velocity=2, acceleration=3)
        )
    duration = time.perf_counter() - start
    print(f"{duration / iterations:.3f} seconds per iteration")
    device._port.close()
