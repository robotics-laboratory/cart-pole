import asyncio
import atexit
import json
import logging
import time

from threading import Event, Thread
from foxglove_websocket.server import FoxgloveServer
from mcap.writer import CompressionType, Writer
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

def make_logger(name) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)

    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("[%(name)s] [%(levelname)s] %(asctime)s:  %(message)s"))
    logger.addHandler(handler)

    return logger


class MCAPLogger:
    def __init__(self, log_path: str):
        '''
        Args:
            log_path: path to mcap log file
        '''

        self._log = make_logger('MCAPLogger')

        self._writer = Writer(open(log_path, "wb"))
        self._writer.start()

        self._topic_to_registration: Dict[str, Registration] = {}

    def __enter__(self):
        return self

    def _register(self, topic_name: str, cls: Any) -> Registration:
        assert issubclass(cls, BaseModel), 'Required pydantic model, but got {cls.__name__}'

        if topic_name in self._topic_to_registration:
            cached = self._topic_to_registration[topic_name]
            assert cached.cls == cls, f'Topic {topic} already registered with {cached.cls.__name__}'
            return cached

        schema = cls.schema_json()

        schema_id = self._writer.register_schema(
            name=cls.__name__,
            encoding="jsonschema",
            data=schema.encode())

        channel_id = self._writer.register_channel(
            schema_id=schema_id,
            topic=topic_name,
            message_encoding="json")

        cached = Registration(cls=cls, channel_id=channel_id)
        self._topic_to_registration[topic_name] = cached

        self._log.debug('Registration: "%s"=%i %s=%s', topic_name, channel_id, cls.__name__, schema)

        return cached

    def publish(self, topic: str, obj: BaseModel, stamp: float) -> None:
        '''
        Args:
        * topic: topic name
        * obj: object to dump (pydantic model)
        * stamp: timestamp in nanoseconds (float)
        '''

        registation = self._register(topic, type(obj))
        self._writer.add_message(
                channel_id=registation.channel_id,
                log_time=to_ns(stamp),
                data=obj.json().encode(),
                publish_time=to_ns(stamp))

    def close(self):
        self._writer.finish()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

async def _foxglove_async_entrypoint(queue: asyncio.Queue, stop: Event) -> None:
    logger = make_logger('LogServer')
    async with FoxgloveServer("0.0.0.0", 8765, "CartPole", logger=logger) as server:
        topic_to_registration = {}

        async def register(topic_name, cls) -> Registration:
            assert issubclass(cls, BaseModel), f'Required pydantic model, but got {cls.__name__}'

            if topic_name in topic_to_registration:
                cached = topic_to_registration[topic_name]
                assert cached.cls == cls, f'Topic {topic} already registered with {cached.cls.__name__}'
                return cached

            schema = cls.schema_json()

            spec = {
                "topic": topic_name,
                "encoding": "json",
                "schemaName": cls.__name__,
                "schema": schema,
            }

            channel_id = await server.add_channel(spec)
            cached = Registration(cls=cls, channel_id=channel_id)
            topic_to_registration[topic_name] = cached

            logger.debug('Registration "%s"=%i %s=%s', topic_name, channel_id, cls.__name__, schema)

            return cached

        while not stop.is_set():
            try:
                topic_name, stamp, obj = await asyncio.wait_for(queue.get(), timeout=1.0)
                registration = await register(topic_name, type(obj))
                await server.send_message(registration.channel_id, to_ns(stamp), obj.json().encode())
            except asyncio.TimeoutError:
                pass

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

    def __enter__(self):
        return self

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

    def close(self):
        self._stop.set()
        self._foxlgove_thread.join()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

class Logger:
    def __init__(self, log_path: str = ''):
        '''
        Args:
        * log_path: path to mcap log file, if not provided, no mcap log will be created
        '''

        self._foxglove_log = FoxgloveWebsocketLogger()
        self._mcap_log = None
        if log_path:
            self._mcap_log = MCAPLogger(log_path)

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

    def close(self):
        self._foxglove_log.close()
        if self._mcap_log:
            self._mcap_log.close()

    def __exit__(self):
        self.close()


__logger = None

def setup(log_path: str = '') -> None:
    '''
    Args:
    * log_path: path to mcap log file, if not provided, no mcap log will be created
    '''

    global __logger

    if __logger:
        __logger.close()

    __logger = Logger(log_path)
 

def publish(topic_name: str, obj: BaseModel, stamp: float|None = None) -> None:
    '''
    Args:
    * topic_name: topic name
    * obj: pydantic model
    * stamp: timestamp in nanoseconds (float), if not provided, current time used
    '''

    global __logger

    if not __logger:
        setup()

    if stamp is None:
        stamp = time.time()

    __logger.publish(topic_name, obj, stamp)


def close():
    global __logger

    if __logger:
        __logger.close()

atexit.register(close)