import asyncio
import json
import logging
import time

from threading import Event, Thread
from foxglove_websocket.server import FoxgloveServer
from mcap.writer import Writer
from pydantic import BaseModel
from typing import Any

from cartpole import State


def to_ns(t: float) -> int:
    '''
    Convert time in seconds to nanoseconds
    '''
    return int(t * 1e9)

class Registration(BaseModel):
    cls: Any
    channel_id: int

class MCAPLogger:
    def __init__(self, log_path: str):
        '''
        Args:
            log_path: path to mcap log file
        '''

        self._writer = Writer(open(log_path, "wb"))
        self._topic_to_registration: Dict[str, Registration] = {}

    def _register(self, topic_name: str, cls: Any) -> None:
        assert issubclass(cls, BaseModel), 'Required pydantic model, but got {cls.__name__}'

        if topic_name in self._topic_to_registration:
            cached = self._topic_to_registration[topic_name]
            assert cached.cls == cls, f'Topic {topic} already registered with {cached.cls.__name__}'
            return

        schema_id = self._writer.register_schema(
            name=cls.__name__,
            encoding="jsonschema",
            data=cls.schema_json().encode())

        channel_id = self._writer.register_channel(
            schema_id=schema_id,
            topic=topic_name,
            message_encoding="json")

        self._topic_to_registration[topic_name] = Registration(cls=cls, channel_id=channel_id)

    def publish(self, topic: str, obj: BaseModel, stamp: float) -> None:
        '''
        Args:
        * topic: topic name
        * obj: object to dump (pydantic model)
        * stamp: timestamp in nanoseconds (float)
        '''

        self._register(topic, type(obj))
        self._writer.add_message(
                channel_id=self._topic_to_registration[topic].channel_id,
                log_time=to_ns(stamp),
                data=obj.json().encode(),
                publish_time=to_ns(stamp))

def foxglove_logger() -> logging.Logger:
    logger = logging.getLogger("LogServer")
    logger.setLevel(logging.ERROR)

    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s: [%(levelname)s] %(message)s"))
    logger.addHandler(handler)

    return logger

async def _foxglove_async_entrypoint(queue: asyncio.Queue, stop: Event) -> None:
    async with FoxgloveServer("0.0.0.0", 8765, "CartPole", logger=foxglove_logger()) as server:
        topic_to_registration = {}

        async def register(topic_name, cls):
            assert issubclass(cls, BaseModel), f'Required pydantic model, but got {cls.__name__}'

            if topic_name in topic_to_registration:
                cached = topic_to_registration[topic_name]
                assert cached.cls == cls, f'Topic {topic} already registered with {cached.cls.__name__}'
                return

            spec = {
                "topic": topic_name,
                "encoding": "json",
                "schemaName": cls.__name__,
                "schema": cls.schema_json().encode(),
            }

            channel_id = await server.add_channel(spec)
            topic_to_registration[topic_name] = Registration(cls=cls, channel_id=channel_id)

        while not stop.is_set():
            topic_name, stamp, obj = await queue.get()
            await register(topic_name, type(obj))
            channel_id = topic_to_registration[topic_name].channel_id
            await server.send_message(channel_id, to_ns(stamp), obj.json().encode())

def foxglove_main(loop: asyncio.AbstractEventLoop, queue: asyncio.Queue, stop: Event) -> None:
    asyncio.set_event_loop(loop)
    loop.run_until_complete(_foxglove_async_entrypoint(queue, stop))

class FoxgloveWebsocketLogger:
    def __init__(self):
        self._loop = asyncio.new_event_loop()
        self._queue = asyncio.Queue()
        self._stop = Event()
        self._writer = None

        self._foxlgove_thread = Thread(
            target=foxglove_main,
            name='foxglove_main_loop',
            daemon=True,
            args=(self._loop, self._queue, self._stop))

        self._foxlgove_thread.start()

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        '''
        Args:
        * topic_name: topic name
        * obj: object to dump (pydantic model)
        * stamp: timestamp in nanoseconds (float)
        '''

        if not (self._loop.is_running() and self._foxlgove_thread.is_alive()):
            raise RuntimeError('Foxglove logger is not running')

        item = (topic_name, stamp, obj)
        asyncio.run_coroutine_threadsafe(self._queue.put(item), self._loop)

    def __del__(self):
        self._stop.set()

class Logger:
    def __init__(self, log_path: str = ''):
        '''
        Args:
        * log_path: path to mcap log file, if not provided, no mcap log will be created
        '''

        self._foxglove_log = FoxgloveWebsocketLogger()
        self._mcap_log = None
        if log_path:
            self.mcap_log = MCAPLogger(log_path)

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        '''
        Args:
        * topic_name: topic name
        * obj: pydantic model
        * stamp: timestamp in nanoseconds (float)
        '''

        if self._mcap_log:
            self._mcap_log.publish(topic_name, obj, stamp)

        self._foxglove_log.publish(topic_name, obj, stamp)


__logger = None

def setup(log_path: str = '') -> None:
    '''
    Args:
    * log_path: path to mcap log file, if not provided, no mcap log will be created
    '''

    global __logger
    __logger = Logger(log_path)


def publish(topic_name: str, obj: BaseModel, stamp: float|None = None) -> None:
    '''
    Args:
    * topic_name: topic name
    * obj: pydantic model
    * stamp: timestamp in nanoseconds (float), if not provided, current time used
    '''

    if not __logger:
        setup()

    if stamp is None:
        stamp = time.time()

    __logger.publish(topic_name, obj, stamp)