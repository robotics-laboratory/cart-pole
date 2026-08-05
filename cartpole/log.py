"""MCAP / Foxglove logging helpers used by physical evaluation."""

from __future__ import annotations

import atexit
import enum
import json
import logging
import time
from typing import Any

from pydantic import BaseModel

NANO = 1_000_000_000
FOXGLOVE_LOG_TOPIC = "/log"
FOXGLOVE_LOG_MSG_TYPE = "foxglove.Log"
FOXGLOVE_LOG_SCHEMA = json.dumps(
    {
        "type": "object",
        "properties": {
            "timestamp": {
                "type": "object",
                "properties": {
                    "sec": {"type": "integer"},
                    "nsec": {"type": "integer"},
                },
            },
            "level": {"type": "integer"},
            "message": {"type": "string"},
            "name": {"type": "string"},
            "file": {"type": "string"},
            "line": {"type": "integer"},
        },
    }
)


class Level(enum.IntEnum):
    UNKNOWN = 0
    DEBUG = 1
    INFO = 2
    WARNING = 3
    ERROR = 4
    FATAL = 5


def pylog_level(level: Level) -> int:
    return {
        Level.UNKNOWN: logging.NOTSET,
        Level.DEBUG: logging.DEBUG,
        Level.INFO: logging.INFO,
        Level.WARNING: logging.WARNING,
        Level.ERROR: logging.ERROR,
        Level.FATAL: logging.CRITICAL,
    }[level]


def to_ns(t: float) -> int:
    return int(t * NANO)


def to_stamp(t: float) -> tuple[int, int]:
    ns = to_ns(t)
    return ns // NANO, ns % NANO


def this_or_now(t: float | None) -> float:
    return t if t is not None else time.time()


def get_pylogger(name: str, level: Level) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(pylog_level(level))
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter("[%(name)s] [%(levelname)s] %(message)s")
        )
        logger.addHandler(handler)
    return logger


class MCAPLogger:
    def __init__(self, log_path: str, level: Level = Level.INFO, compress: bool = True):
        from mcap.writer import CompressionType, Writer

        self._pylog = get_pylogger("cartpole.mcap", level)
        self._writer = Writer(
            open(log_path, "wb"),
            compression=(
                CompressionType.ZSTD if compress else CompressionType.NONE
            ),
        )
        self._writer.start()
        self._topics: dict[str, tuple[str, int]] = {}
        self._log_channel = self._register(
            FOXGLOVE_LOG_TOPIC,
            FOXGLOVE_LOG_MSG_TYPE,
            FOXGLOVE_LOG_SCHEMA,
        )

    def _register(self, topic_name: str, name: str, schema: str) -> int:
        if topic_name in self._topics:
            cached_name, channel_id = self._topics[topic_name]
            assert cached_name == name
            return channel_id
        schema_id = self._writer.register_schema(
            name=name,
            encoding="jsonschema",
            data=schema.encode(),
        )
        channel_id = self._writer.register_channel(
            schema_id=schema_id,
            topic=topic_name,
            message_encoding="json",
        )
        self._topics[topic_name] = (name, channel_id)
        return channel_id

    def _register_class(self, topic_name: str, cls: type[BaseModel]) -> int:
        return self._register(
            topic_name,
            cls.__name__,
            json.dumps(cls.model_json_schema()),
        )

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        channel_id = self._register_class(topic_name, type(obj))
        self._writer.add_message(
            channel_id=channel_id,
            log_time=to_ns(stamp),
            data=obj.model_dump_json().encode(),
            publish_time=to_ns(stamp),
        )

    def log(self, msg: str, stamp: float, level: Level) -> None:
        sec, nsec = to_stamp(stamp)
        payload = {
            "timestamp": {"sec": sec, "nsec": nsec},
            "level": int(level),
            "message": msg,
            "name": "cartpole",
            "file": "/dev/null",
            "line": 0,
        }
        self._writer.add_message(
            channel_id=self._log_channel,
            log_time=to_ns(stamp),
            data=json.dumps(payload).encode(),
            publish_time=to_ns(stamp),
        )

    def close(self) -> None:
        self._writer.finish()


class Logger:
    def __init__(self, log_path: str = "", level: Level = Level.INFO):
        self._pylog = get_pylogger("cartpole", level)
        self._mcap_log = MCAPLogger(log_path, level=level) if log_path else None

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        if self._mcap_log is not None:
            self._mcap_log.publish(topic_name, obj, stamp)

    def log(self, msg: str, stamp: float, level: Level = Level.INFO) -> None:
        self._pylog.log(pylog_level(level), f"{stamp:.3f}: {msg}")
        if self._mcap_log is not None:
            self._mcap_log.log(msg, stamp, level)

    def close(self) -> None:
        if self._mcap_log is not None:
            self._mcap_log.close()
            self._mcap_log = None


__logger: Logger | None = None


def setup(log_path: str = "", level: Level = Level.INFO) -> None:
    global __logger
    close()
    __logger = Logger(log_path=log_path, level=level)


def close() -> None:
    global __logger
    if __logger is not None:
        __logger.close()
        __logger = None


atexit.register(close)


def get_logger() -> Logger:
    global __logger
    if __logger is None:
        setup()
    assert __logger is not None
    return __logger


def publish(topic_name: str, obj: BaseModel, stamp: float | None = None) -> None:
    get_logger().publish(topic_name, obj, this_or_now(stamp))


def log(msg: str, stamp: float | None = None, level: Level = Level.INFO) -> None:
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
