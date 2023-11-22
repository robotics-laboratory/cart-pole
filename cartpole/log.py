import asyncio
import atexit
import enum
import json
import logging
import time
import jsonref

from threading import Event, Thread
from foxglove_websocket.server import FoxgloveServer
from mcap.writer import CompressionType, Writer
from pydantic import BaseModel
from typing import Any, Dict
import os


class Level(enum.IntEnum):
    UNKNOWN = 0
    DEBUG = 1
    INFO = 2
    WARNING = 3
    ERROR = 4
    FATAL = 5


def pylog_level(level: Level) -> int:
    if level == Level.UNKNOWN:
        return logging.NOTSET
    elif level == Level.DEBUG:
        return logging.DEBUG
    elif level == Level.INFO:
        return logging.INFO
    elif level == Level.WARNING:
        return logging.WARNING
    elif level == Level.ERROR:
        return logging.ERROR
    elif level == Level.FATAL:
        return logging.CRITICAL
    else:
        raise ValueError(f"Unknown log level {level}")


FOXGLOVE_LOG_TOPIC = "/log"

FOXGLOVE_LOG_MSG_TYPE = "foxglove.Log"

FOXGLOVE_LOGM_SG_SCHEMA = """
{
  "title": "foxglove.Log",
  "description": "A log message",
  "$comment": "Generated by https://github.com/foxglove/schemas",
  "type": "object",
  "properties": {
    "timestamp": {
      "type": "object",
      "title": "time",
      "properties": {
        "sec": {
          "type": "integer",
          "minimum": 0
        },
        "nsec": {
          "type": "integer",
          "minimum": 0,
          "maximum": 999999999
        }
      },
      "description": "Timestamp of log message"
    },
    "level": {
      "title": "foxglove.Level",
      "description": "Log level",
      "oneOf": [
        {
          "title": "UNKNOWN",
          "const": 0
        },
        {
          "title": "DEBUG",
          "const": 1
        },
        {
          "title": "INFO",
          "const": 2
        },
        {
          "title": "WARNING",
          "const": 3
        },
        {
          "title": "ERROR",
          "const": 4
        },
        {
          "title": "FATAL",
          "const": 5
        }
      ]
    },
    "message": {
      "type": "string",
      "description": "Log message"
    },
    "name": {
      "type": "string",
      "description": "Process or node name"
    },
    "file": {
      "type": "string",
      "description": "Filename"
    },
    "line": {
      "type": "integer",
      "minimum": 0,
      "description": "Line number in the file"
    }
  }
}
"""


NANO: int = 1000000000


def to_ns(t: float) -> int:
    """
    Convert time in seconds (float) to nanoseconds (int)
    """
    return int(t * NANO)


def to_stamp(t: int) -> tuple[int, int]:
    """
    Convert time in nanoseconds to (seconds, nanoseconds).
    """
    return divmod(t, NANO)


def this_or_now(t: float | None) -> float:
    """
    Return provided time or current time
    """
    return t if t is not None else time.perf_counter()


class Registration(BaseModel):
    name: str
    channel_id: int


def get_pylogger(name: str, level: Level) -> logging.Logger:
    """
    Create python or get existing logger. Log Level is translated from foxglove to python.

    Args:
        name: logger name
        level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
    """

    logger = logging.getLogger(name)
    logger.setLevel(pylog_level(level))

    # naive check that logger is configured
    if not logger.hasHandlers():
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter("[%(name)s] [%(levelname)s] %(message)s")
        )
        logger.addHandler(handler)

    return logger


# JSON schema helper functions, TODO: preffity


def _remove_allof(schema):
    if not isinstance(schema, dict):
        return schema
    if "allOf" in schema:
        items = schema.pop("allOf")
        assert len(items) == 1, "Union types are not supported"
        assert isinstance(items[0], dict), "allOf members must be dicts"
        schema.update(items[0])
    return {key: _remove_allof(value) for key, value in schema.items()}


def _model_to_schema(model: BaseModel) -> str:
    schema = jsonref.replace_refs(model.schema(), proxies=False)
    schema = _remove_allof(schema)
    schema = jsonref.dumps(schema)
    return schema


class MCAPLogger:
    """
    Logger to mcap file.

    Usage:
        with MCAPLogger(log_path='log.mcap') as log:
            obj = ... # some pydantic object
            log.publish('/topic', obj, stamp)
            log.info('message')
    """

    def __init__(self, log_path: str, level: Level = Level.INFO, compress=True):
        """
        Args:
            log_path: path to mcap log file
            level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
            compress: enable compression
        """

        self._pylog = get_pylogger("cartpole.mcap", level)

        self._writer = Writer(
            output=open(log_path, "wb"),
            compression=CompressionType.ZSTD if compress else CompressionType.NONE,
        )

        self._writer.start()
        self._topic_to_registration: Dict[str, Registration] = {}

        # preventive topic creation
        self.registration_log = self._register(
            FOXGLOVE_LOG_TOPIC, FOXGLOVE_LOG_MSG_TYPE, FOXGLOVE_LOGM_SG_SCHEMA
        )

    def __enter__(self):
        return self

    def _register(self, topic_name: str, name: str, schema: str) -> Registration:
        if topic_name in self._topic_to_registration:
            cached = self._topic_to_registration[topic_name]
            assert (
                cached.name == name
            ), f"Topic {topic_name} registered with {cached.name}"
            return cached

        schema_id = self._writer.register_schema(
            name=name, encoding="jsonschema", data=schema.encode()
        )

        channel_id = self._writer.register_channel(
            schema_id=schema_id, topic=topic_name, message_encoding="json"
        )

        cached = Registration(name=name, channel_id=channel_id)
        self._topic_to_registration[topic_name] = cached
        self._pylog.debug("id=%i topic='%s', type='%s'", channel_id, topic_name, name)

        return cached

    def _register_class(self, topic_name: str, cls: Any) -> Registration:
        name = cls.__name__
        if name in self._topic_to_registration:
            return self._topic_to_registration[name]
        assert issubclass(cls, BaseModel), "Required pydantic model, but got {name}"
        schema = _model_to_schema(cls)
        return self._register(topic_name, name, schema)

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        """
        Publish object to topic.

        Args:
            topic_name: topic name
            obj: object to dump (pydantic model)
            stamp: timestamp in seconds (float)
        """

        registation = self._register_class(topic_name, type(obj))
        stamp_ns = to_ns(stamp)
        self._writer.add_message(
            channel_id=registation.channel_id,
            log_time=stamp_ns,
            data=obj.json().encode(),
            publish_time=stamp_ns,
        )

    def log(self, msg: str, stamp: float, level: Level) -> None:
        """
        Print message to topic /log.

        Args:
            msg: message to print
            stamp: timestamp in seconds (float)
            level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
        """

        stamp_ns = to_ns(stamp)
        sec, nsec = to_stamp(stamp_ns)

        obj = {
            "timestamp": {"sec": sec, "nsec": nsec},
            "level": int(level),
            "message": msg,
            "name": "cartpole",
            "file": "/dev/null",
            "line": 0,
        }

        self._writer.add_message(
            channel_id=self.registration_log.channel_id,
            log_time=stamp_ns,
            data=json.dumps(obj).encode(),
            publish_time=stamp_ns,
        )

    def debug(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.DEBUG)

    def info(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.INFO)

    def warning(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.WARNING)

    def error(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.ERROR)

    def fatal(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.FATAL)

    def close(self):
        self._writer.finish()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class FoxgloveWebsocketLogger:
    def __init__(self, level: int = Level.INFO):
        self._loop = asyncio.new_event_loop()
        # self._input_queue = asyncio.Queue()
        self._input_queue = []
        self._exception_queue = asyncio.Queue()
        self._stop = Event()

        self._server = None
        self._topic_to_registration = {}

        self._thread = Thread(
            target=self.foxglove_main,
            name="foxglove_main_loop",
            daemon=True,
        )
        self._thread.start()

    def foxglove_main(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._foxglove_async_wrapping())

    async def _foxglove_async_wrapping(self):
        try:
            await self._foxglove_async_entrypoint()
        except Exception as e:
            await self._exception_queue.put(e)

    async def _register(self, topic_name, name, schema) -> Registration:
        spec = {
            "topic": topic_name,
            "encoding": "json",
            "schemaName": name,
            "schema": schema,
        }

        channel_id = await self._server.add_channel(spec)
        cached = Registration(name=name, channel_id=channel_id)
        self._topic_to_registration[topic_name] = cached

        # logger.debug("id=%i topic='%s', type='%s'", channel_id, topic_name, name)
        return cached

    async def _register_class(self, topic_name, cls) -> Registration:
        name = cls.__name__
        if topic_name in self._topic_to_registration:
            cached = self._topic_to_registration[topic_name]
            assert (
                cached.name == name
            ), f"Topic {topic_name} registered with {cached.name}"
            return cached
        assert issubclass(cls, BaseModel), f"Required pydantic model, but got {name}"
        schema = _model_to_schema(cls)
        return await self._register(topic_name, name, schema)

    async def _foxglove_async_entrypoint(self):
        # logger = get_pylogger("cartpole.foxglove", level)

        async with FoxgloveServer("0.0.0.0", 8765, "CartPole") as self._server:
            # preventive topic creation
            # registration_log = await self._register(
            #     FOXGLOVE_LOG_TOPIC, FOXGLOVE_LOG_MSG_TYPE, FOXGLOVE_LOGM_SG_SCHEMA
            # )

            while not self._stop.is_set():
                await asyncio.sleep(0.1)
                # try:
                #     if not len(queue):
                #         await asyncio.sleep(0.01)
                #         continue
                #     topic_name, stamp, obj = queue.pop(0)
                #     stamp_ns = to_ns(stamp)
                #     sec, nsec = to_stamp(stamp_ns)

                #     if topic_name == FOXGLOVE_LOG_TOPIC:
                #         # special logic for logs
                #         assert isinstance(obj, str), f'Expected string, but got {type(obj)}'
                #         registration = registration_log

                #         msg = {
                #             "timestamp": {"sec": sec, "nsec": nsec},
                #             "level": int(level),
                #             "message": obj,
                #             "name": "cartpole",
                #             "file": "/dev/null",
                #             "line": 0
                #         }

                #         data = json.dumps(msg).encode()
                #     else:
                #         if topic_name in topic_to_registration:
                #             registration = topic_to_registration[topic_name]
                #         else:
                #             registration = await register_class(topic_name, type(obj))
                #         data = obj.json().encode()

                #     await server.send_message(registration.channel_id, stamp_ns, data)

                # except asyncio.TimeoutError:
                #     pass

    async def _direct_publish(self, topic_name, stamp, obj):
        reg = await self._register_class(topic_name, type(obj))
        data = obj.json().encode()
        await self._server.send_message(reg.channel_id, to_ns(stamp), data)

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        if not (self._loop.is_running() and self._thread.is_alive()):
            if not self._exception_queue.empty():
                raise self._exception_queue.get_nowait()
            raise AssertionError("Foxglove logger is not running")

        # item = (topic_name, stamp, obj)
        # self._input_queue.append(item)
        # asyncio.ensure_future(self._direct_publish(topic_name, stamp, obj), loop=self._loop)
        asyncio.run_coroutine_threadsafe(
            self._direct_publish(topic_name, stamp, obj), self._loop
        )

    def log(self, msg: str, stamp: float, level: int) -> None:
        self.publish("/log", msg, stamp)

    def debug(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.DEBUG)

    def info(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.INFO)

    def warning(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.WARNING)

    def error(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.ERROR)

    def fatal(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.FATAL)

    def close(self):
        self._stop.set()
        self._thread.join()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class Logger:
    """
    Compound Logger class that logs to console, foxglove and mcap.

    Usage:
        with Logger(log_path='log.mcap', level=INFO) as log:
            obj = ... # some pydantic object

            log.publish('/topic', obj)
            log.info('message')
    """

    def __init__(self, log_path: str = "", level: Level = Level.INFO):
        """
        Args:
        * log_path: path to mcap log file, if not provided, no mcap log will be created
        * level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
        """

        self._pylog = get_pylogger("cartpole", level)
        self._foxglove_log = None
        # TODO: Prettify
        # if os.environ.get("FOXGLOVE_ENABLE", "false").lower() == "true":
        #     self._foxglove_log = FoxgloveWebsocketLogger()
        self._mcap_log = None

        if log_path:
            self._mcap_log = MCAPLogger(log_path, level=level)

    def publish(self, topic_name: str, obj: BaseModel, stamp: float = None) -> None:
        """
        Args:
            topic_name: topic name
            obj: pydantic object
            stamp: timestamp in seconds (float), if not provided, current time used
        """
        if stamp is None:
            stamp = time.perf_counter()

        if self._mcap_log:
            self._mcap_log.publish(topic_name, obj, stamp)

        if self._foxglove_log:
            self._foxglove_log.publish(topic_name, obj, stamp)

    def log(self, msg: str, stamp: float, level: Level = Level.INFO) -> None:
        """
        Print message to console and topic /log.

        Args:
            msg: message to print
            stamp: timestamp in nanoseconds (float), if not provided, current time used
            level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
        """

        self._pylog.log(pylog_level(level), f"{stamp:.3f}: {msg}")
        if self._foxglove_log:
            self._foxglove_log.log(msg, stamp, level)

        if self._mcap_log:
            self._mcap_log.log(msg, stamp, level)

    def debug(self, msg: str, stamp: float) -> None:
        self.info(msg, stamp, Level.DEBUG)

    def info(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.INFO)

    def warning(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.WARNING)

    def error(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.ERROR)

    def fatal(self, msg: str, stamp: float) -> None:
        self.log(msg, stamp, Level.FATAL)

    def close(self):
        """
        Close logger
        """
        if self._foxglove_log:
            self._foxglove_log.close()
        if self._mcap_log:
            self._mcap_log.close()

    def __exit__(self):
        self.close()


__logger = None


def setup(log_path: str = "", level: Level = Level.INFO) -> None:
    """
    Args:
        log_path: path to mcap log file, if not provided, no mcap log will be created
        level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
    """

    global __logger
    close()
    __logger = Logger(log_path=log_path, level=level)


def close():
    """
    Close logger
    """

    global __logger

    if __logger:
        __logger.close()
        __logger = None


atexit.register(close)


def get_logger() -> Logger:
    """
    Get logger instance
    """

    global __logger
    if not __logger:
        setup()

    return __logger


def publish(topic_name: str, obj: BaseModel, stamp: float | None = None) -> None:
    """
    Args:
        topic_name: topic name
        obj: pydantic model
        stamp: timestamp in nanoseconds (float), if not provided, current time used
    """

    get_logger().publish(topic_name, obj, this_or_now(stamp))


def log(msg: str, stamp: float | None = None, level: Level = Level.INFO) -> None:
    """
    Print message to console and topic /log.

    Args:
        msg: message to print
        stamp: timestamp in nanoseconds (float), if not provided, current time used
        level: log level (UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL)
    """

    get_logger().log(msg, this_or_now(stamp), level)


def debug(msg: str, stamp: float | None = None) -> None:
    log(msg, stamp, Level.DEBUG)


def info(msg: str, stamp: float | None = None) -> None:
    log(msg, stamp, Level.INFO)


def warning(msg: str, stamp: float | None = None) -> None:
    log(msg, stamp, Level.WARNING)


def error(msg: str, stamp: float | None = None) -> None:
    log(msg, stamp, Level.ERROR)


def fatal(msg: str, stamp: float | None = None) -> None:
    log(msg, stamp, Level.FATAL)
