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

from betterproto import Message
from pydantic import BaseModel, BaseSettings, FilePath
from serial import Serial
from serial.tools.list_ports import comports

from cartpole.common import CartPoleBase
from cartpole.common.interface import Config, Error, State
from cartpole.device import framing


class ConnectionSettings(BaseSettings):
    serial_port: FilePath = None
    serial_speed: int = 3_000_000
    serial_timeout: float = 0.1
    hard_reset: bool = True

    @classmethod
    def with_overrides(cls, **kwargs):
        kwargs = {k: v for k, v in kwargs.items() if v is not None}
        return cls(**kwargs)


class TypeMapping(NamedTuple):
    request_native_type: Optional[BaseModel]
    request_proto_type: Optional[Message]
    response_native_type: Optional[BaseModel]
    response_proto_type: Optional[Message]


# TODO: Move to cartpole.common.interface?
class Target(BaseModel):
    position: Optional[float] = None
    velocity: Optional[float] = None
    acceleration: Optional[float] = None


class DeviceConfig(Config, proto.Config):
    pass  # Use protobuf type as-is (for now)


class CartPoleDevice(CartPoleBase):
    REQUEST_MAPPING = {
        RequestType.RESET: TypeMapping(
            request_native_type=None,
            request_proto_type=None,
            response_native_type=None,
            response_proto_type=None,
        ),
        # TODO: Custom DeviceState class?
        RequestType.TARGET: TypeMapping(
            request_native_type=Target,
            request_proto_type=proto.Target,
            response_native_type=State,
            response_proto_type=proto.State,
        ),
        RequestType.CONFIG: TypeMapping(
            request_native_type=DeviceConfig,
            request_proto_type=proto.Config,
            response_native_type=DeviceConfig,
            response_proto_type=proto.Config,
        ),
    }
    RESET_DELAY = 0.1
    HOMING_TIMEOUT = 10

    def __init__(self, **kwargs):
        cfg = ConnectionSettings.with_overrides(**kwargs)
        if cfg.serial_port is None:
            cfg.serial_port = self._detect_port()
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
        logging.info(f"Purged data: {data.decode(errors='ignore')}")

    def _request(self, type: RequestType, payload: Union[BaseModel, Message] = None):
        logging.debug(f"SEND: {type!r} request with payload: {payload}")
        mapping = self.REQUEST_MAPPING[type]
        if payload is not None:
            if not isinstance(payload, Message):
                payload = mapping.request_proto_type(**payload.dict())
            assert isinstance(payload, mapping.request_proto_type)
        data = framing.encode(type)
        self._port.write(data)
        logging.debug(f"SEND (raw): {data}")
        data = self._port.read_until(framing.FRAME_DELIMITER)
        logging.debug(f"RECV (raw): {data}")
        if not data:
            logging.error(f"{type!r} request timeout")
            raise TimeoutError
        response_type, response = framing.decode(data, mapping.response_proto_type)
        assert type == response_type, "Unexpected response type"
        if response is not None:
            response = mapping.request_native_type(**response.as_dict())
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

    iterations = 2000
    start = time.perf_counter()
    device = CartPoleDevice()
    for _ in range(iterations):
        device._request(RequestType.RESET)
    duration = time.perf_counter() - start
    print(f"{duration / iterations:.3f} seconds per iteration")
    device._port.close()
