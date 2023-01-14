from cartpole import State, Logger
from pydantic import BaseModel
from time import sleep, time

import pytest

class SomeObject(BaseModel):
    name: str = ''
    value: float = 0.0


def test_log_some_object():
    log = Logger('test.mcap')
    log('/some/object', time(), SomeObject(name='first', value=1.0))
    log('/some/object', time(), SomeObject(name='second', value=2.0))

def test_log_bad_object():
    log = Logger('test.mcap')

    # send bad object (not pydantic model)
    log('/str', time(), 'something')

    # after a while get an error on next call
    sleep(0.1)
    with pytest.raises(RuntimeError):
        log('/some/object', time(), SomeObject())


def test_log_state():
    log= Logger('test.mcap')
    log('/cartpole/state', time(), State())


def test_log_different_types_to_one_topic():
    topic = '/cartpole/state'
    log = Logger('test.mcap')

    log(topic, time(), State())

    # send another type to the same topic
    log(topic, time(), SomeObject())

    # after a while get an error on next call
    sleep(0.1)
    with pytest.raises(RuntimeError):
        log(topic, time(), State())

