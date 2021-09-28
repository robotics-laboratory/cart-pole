import dataclasses as dc
import json
import logging
import numpy as np
import inspect
import string
import threading
import time
from contextlib import contextmanager
from collections import defaultdict
from typing import List, Dict, Type

from interface import CartPoleBase, Config, State
from sessions.actor import Actor


LOGGER = logging.getLogger(__name__)
HEX_DIGITS = list(string.digits + string.ascii_lowercase[:6])


def generate_id(size: int = 6):
    return ''.join(np.random.choice(HEX_DIGITS, size=size, replace=True))

def micros() -> int:
    return time.time_ns() // 1000

def perf_counter_micros() -> int:
    return time.perf_counter_ns() // 1000

def serialize(v):
    if dc.is_dataclass(v):
        res = {}
        for field in dc.fields(v):
            res[field.name] = serialize(getattr(v, field.name))
        return res
    elif isinstance(v, list):
        return [serialize(item) for item in v]
    elif isinstance(v, dict):
        return {key: serialize(value) for key, value in v.items()}
    else:
        return v

class Units:
    METERS = 'm'
    METERS_PER_SECOND = 'm/s'
    METERS_PER_SECOND_SQUARED = 'm/s^2'
    RADIANS = 'rad'
    RADIANS_PER_SECOND = 'rad/s'


@dc.dataclass
class SessionData:
    @dc.dataclass
    class SessionMeta:
        session_id: str = dc.field(default_factory=generate_id)
        name: str = None
        start_timestamp: int = None
        duration: int = None
        device_class: str = None
        device_config: Config = None
        actor_class: str = None
        actor_config: dict = None

    @dc.dataclass
    class Log:
        timestamp: int
        message: str

    @dc.dataclass
    class Value:
        id: str
        name: str
        unit: str
        x: List[float] = dc.field(default_factory=list, repr=False)
        y: List[float] = dc.field(default_factory=list, repr=False)

    @dc.dataclass
    class Group:
        name: str
        values: List[str] = dc.field(default_factory=list)

    @dc.dataclass
    class Style:
        name: str
        # todo

    @dc.dataclass
    class TimeTrace:
        action: str
        start_timestamp: int = dc.field(default_factory=micros)
        start_perf_counter: int = dc.field(default_factory=perf_counter_micros)
        finish_timestamp: int = None
        finish_perf_counter: int = None

    meta: SessionMeta = dc.field(default_factory=SessionMeta)
    values: Dict[str, Value] = dc.field(default_factory=dict)
    groups: List[Group] = dc.field(default_factory=list)
    logs: List[Log] = dc.field(default_factory=list)
    time_traces: List[TimeTrace] = dc.field(default_factory=list)

    def __post_init__(self) -> None:
        V = SessionData.Value
        values = [
            V(id='state.position', name='position', unit=Units.METERS),
            V(id='state.velocity', name='velocity', unit=Units.METERS_PER_SECOND),
            V(id='state.acceleration', name='acceleration', unit=Units.METERS_PER_SECOND_SQUARED),
            V(id='state.pole_angle', name='angle', unit=Units.RADIANS),
            V(id='state.pole_velocity', name='angle', unit=Units.RADIANS_PER_SECOND),
            V(id='target.acceleration', name='acceleration', unit=Units.RADIANS_PER_SECOND),
        ]
        self.values = {v.id: v for v in values}
        self.groups = [
            # todo
        ]

    def to_dict(self) -> dict:
        return serialize(self)


class CollectorProxy(CartPoleBase):
    def __init__(self, cart_pole: CartPoleBase, actor_class: Type[Actor], actor_config: dict) -> None:
        self.cart_pole = cart_pole
        self.actor_class = actor_class
        self.actor_config = actor_config
        self.data: SessionData = None

        self._locks = defaultdict(threading.Lock)
        
        self._started_flag = threading.Event()
        self._available_data = []
        self._consumer_flag = threading.Event()
        self._is_finished = False

        
    def save(self):
        with open(f'{self.data.meta.session_id}.json', 'w') as f:
            json.dump(self.data.to_dict(), f, indent=4)

    @contextmanager
    def time_trace(self, action=None):
        action = action or inspect.stack()[2][3]
        trace = SessionData.TimeTrace(action=action)
        yield trace
        trace.finish_timestamp = micros()
        trace.finish_perf_counter = perf_counter_micros()
        self.data.time_traces.append(trace)

    def reset(self, config: Config) -> None:
        with self.time_trace():
            self.data = SessionData(meta=SessionData.SessionMeta(
                device_config=config,
                actor_class=self.actor_class.__name__,
                actor_config=self.actor_config,
            ))

            self.cart_pole.reset(config)
            self.data.meta.start_timestamp = micros()
            self._started_flag.set()

    def get_state(self) -> State:
        with self.time_trace() as trace:
            state = self.cart_pole.get_state()

            for field in dc.fields(state):
                key = f'state.{field.name}'
                if key not in self.data.values:
                    continue  # TODO?: maybe raise error
                
                value = getattr(state, field.name)
                if value is None:
                    continue

                self.data.values[key].x.append(trace.start_timestamp)
                self.data.values[key].x.append(value)

            return state

    def get_info(self) -> dict:
        with self.time_trace():
            return self.cart_pole.get_info()

    def get_target(self) -> float:
        with self.time_trace():
            return self.cart_pole.get_target()

    def set_target(self, target: float) -> None:
        with self.time_trace() as trace:
            key = 'target.acceleration'
            self.data.values[key].x.append(trace.start_timestamp)
            self.data.values[key].y.append(target)
            
            return self.cart_pole.set_target(target)

    def make_step(self) -> None:
        with self.time_trace():
            self.cart_pole.make_step()

    def close(self) -> None:
        with self.time_trace():
            self.cart_pole.close()
            self.data.meta.duration = micros() - self.data.meta.start_timestamp
            self.save()
            self._started_flag.clear()
            self.data = None


    def meta(self) -> dict:
        if not self._started_flag.is_set():
            raise RuntimeError('Session has not started yet')
        return self.data.to_dict()

    def consume_value(self) -> SessionData.Value:
        pass


if __name__ == '__main__':
    class FakeCartPole(CartPoleBase):
        def reset(self, config: Config) -> None:
            pass

        def get_state(self) -> State:
            return State(position=1)
        
        def set_target(self, target: float) -> None:
            pass

        def close(self) -> None:
            pass

    logging.getLogger().setLevel(logging.DEBUG)
    c = CollectorProxy(FakeCartPole())
    c.reset(Config())
    time.sleep(2)
    c.set_target(1)
    time.sleep(2)
    c.set_target(2)
    time.sleep(1)
    c.set_target(3)
    c.close()

    # print(len(c.consume('state.position', 0)))  # FIXME

    c.reset(Config())
    time.sleep(5)
    c.close()
