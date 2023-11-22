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
from pydantic import BaseModel, BaseSettings
from pathlib import Path
from serial import Serial
from serial.tools.list_ports import comports

from cartpole.common import CartPoleBase, State, Target, Config, Error
from cartpole.device import framing
import threading


class ConnectionSettings(BaseSettings):
    """
    Device connection settings. Can be set via environment variables, or overriden
    directly by passing kwargs to CartPoleDevice constructor.

    - serial_port: path to serial device (e.g. /dev/serial/by-id/xxx)
    - serial_speed: serial baud rate (bits/s)
    - serial_timeout: serial read timeout (s)
    - hard_reset: whether to perform hard device reset upon connecting
    """

    serial_port: Path = None
    serial_speed: int = 500000
    serial_timeout: float = 0.2
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


class DeviceState(State):
    hardware_errors: int


class DeviceConfig(Config, proto.Config):
    pass  # Use protobuf type as-is (for now)


class CartPoleDevice(CartPoleBase):
    REQUEST_MAPPING = {
        # Reset command: accepts nothing, returns state
        RequestType.RESET: TypeMapping(None, None, State, proto.State),
        # Set target command: accepts target, returns state
        RequestType.TARGET: TypeMapping(Target, proto.Target, DeviceState, proto.State),
        # Set config command: accepts config, returns config
        RequestType.CONFIG: TypeMapping(
            DeviceConfig, proto.Config, proto.Config, proto.Config
        ),
    }
    RESET_DELAY = 0.3
    HOMING_TIMEOUT = 10

    def __init__(self, **kwargs):
        cfg = ConnectionSettings.with_overrides(**kwargs)
        cfg.serial_port = cfg.serial_port or self._detect_port()
        logging.info(f"Serial: {cfg.serial_port} @ {cfg.serial_speed} bits/s")
        self._port = Serial(
            port=str(cfg.serial_port),
            baudrate=cfg.serial_speed,
            timeout=cfg.serial_timeout,
            exclusive=True,
        )
        if cfg.hard_reset:
            self._hard_reset()
        self._request_lock = threading.Lock()
        self._last_state = None
        self._last_state_time = time.perf_counter()

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
        data = self._port.read_all()
        logging.debug(f"Purged data: {data.decode(errors='ignore')!r}")

    def _request(self, type: RequestType, payload: Union[BaseModel, Message] = None):
        logging.debug(f"SEND: {type!r} request with payload: {payload}")
        mapping = self.REQUEST_MAPPING[type]
        if payload is not None:
            if not isinstance(payload, Message):
                payload = mapping.req_message(**payload.dict())
            assert isinstance(payload, mapping.req_message)
        data = framing.encode(type, payload)
        self._port.write(data)
        logging.debug(f"SEND (raw): {data}")
        data = self._port.read_until(framing.FRAME_DELIMITER)
        logging.debug(f"RECV (raw): {data}")
        if not data:
            logging.error(f"{type!r} request timeout")
            raise TimeoutError
        try:
            response_type, response = framing.decode(data, mapping.res_message)
        except Exception as err:
            raise RuntimeError(f"Failed to decode:\n{data!r}") from err
        assert type == response_type, "Unexpected response type"
        if response is not None:
            response = mapping.res_model(**response.to_dict(Casing.SNAKE))
        logging.debug(f"RECV: {response_type} response with payload: {response}")
        return response

    def reset(self, state: State = State()):
        self._request(RequestType.RESET)
        start = time.perf_counter()
        state: State = self.get_state(force=True)
        while state.error != Error.NO_ERROR:
            state = self.get_state()
            if time.perf_counter() - start > self.HOMING_TIMEOUT:
                raise RuntimeError(f"Device homing timeout. Last known state: {state}")
        # time.sleep(5.0)
        # if state.cart_position != 0:
        #     # TODO: Move to position (if specified)
        #     raise NotImplementedError

    def _push_state(self, state: DeviceState):
        self._last_state = state
        self._last_state_time = time.perf_counter()

    def set_target(self, target: Target) -> State:
        state = self._request(RequestType.TARGET, target)
        self._push_state(state)
        return state

    def get_state(self, force: bool = False) -> State:
        # if self._last_state is not None and not force:
        #     # with self._lock:
        #     if time.perf_counter() - self._last_state_time < 0.02:
        #         return self._last_state
        state = self._request(RequestType.TARGET, Target())
        # self._push_state(state)
        return state

    def set_config(self, config: Config):
        print("set_config not implemented (fixme)")
        pass

    def get_config(self) -> Config:
        return self._request(RequestType.CONFIG, proto.Config())

    def advance(self, delta: float):
        pass


if __name__ == "__main__":
    from logging import basicConfig

    basicConfig(level="INFO")

    ### SIMPLE COMM TEST
    # device = CartPoleDevice()
    # target = Target(position=1, velocity=2, acceleration=3)
    # state = device.set_target(target)
    # print(state)

    ### ENCODER TEST + FOXGLOVE
    from cartpole import log

    log.setup(log_path="untracked/realtime_true.mcap")
    logger = log.get_logger()
    device = CartPoleDevice(hard_reset=True)

    # def state_update_loop():
    #     while True:
    #         state = device.get_state()
    #         # print(f"errors: {state.error}, flags: {bin(state.hardware_errors)}")
    #         logger.publish("/cartpole/state", state)
    #         time.sleep(1 / 100)

    # t = threading.Thread(target=state_update_loop, daemon=True)
    # t.start()

    state = device.get_state()
    device.reset()
    time.sleep(5)

    def wait_position_reached(target: Target):
        state = device.get_state()
        while abs(target.position - state.cart_position) > 0.001:
            # time.sleep(0.01)
            state = device.get_state()
            logger.publish("/cartpole/state", state)

    x = 0.15
    v = 1.0
    a = 2.0

    for a in [0.1, 0.2, 0.5, 1.0]:
        target = Target(position=-x, velocity=v, acceleration=a)
        state = device.set_target(target)
        logger.publish("/cartpole/state", state)
        wait_position_reached(target)

        target = Target(position=+x, velocity=v, acceleration=a)
        state = device.set_target(target)
        logger.publish("/cartpole/state", state)
        wait_position_reached(target)

    # target = Target(position=0, velocity=v, acceleration=0.1)
    # device.set_target(target)
    # wait_position_reached(target)

    # device._request(RequestType.RESET)
    # time.sleep(10)

    # times = []
    # min_rps = float("inf")
    # max_rps = float("-inf")
    # while True:
    #     start = time.perf_counter()
    #     state: DeviceState = device.set_target(Target(position=1, velocity=2, acceleration=3))
    #     duration = time.perf_counter() - start
    #     min_rps = min(min_rps, 1 / duration)
    #     max_rps = max(max_rps, 1 / duration)
    #     times.append(duration)
    #     times = times[-100:]
    #     mean_rps = 1 / (sum(times) / len(times))
    #     # print(f"{state.pole_angle:.3f}, {state.pole_angular_velocity:.3f} ({mean_rps:.0f} Hz, {min_rps:.0f} min, {max_rps:.0f} max)")
    #     print(f"{state.pole_angle:.3f}, {state.pole_angular_velocity:.3f}, {state.hardware_error}")
    #     logger.publish("/cartpole/state", state)

    ### PERF TEST
    # iterations = 1000
    # start = time.perf_counter()
    # device = CartPoleDevice()
    # for _ in range(iterations):
    #     state = device._request(
    #         RequestType.TARGET, Target(position=1, velocity=2, acceleration=3)
    #     )
    # duration = time.perf_counter() - start
    # print(f"{duration / iterations:.3f} seconds per iteration")
    # device._port.close()
