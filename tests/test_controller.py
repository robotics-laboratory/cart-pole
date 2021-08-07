import functools
import mock
import pytest

from cart_pole import controller, util
from cart_pole.interface import State, action
from cart_pole.serial_connection import SerialConnection


class MockSerial:
    def __init__(self, with_error=False, *args, **kwargs) -> None:
        self.last_wrote: str = ''
        self.error = with_error
        self.name = 'lol'
        self.has_stepped = False

    def write(self, s: str) -> None:
        self.last_wrote = s
        if s.startswith('state set'):
            self.has_stepped = True

    def readline(self) -> str:
        if self.error:
            return '!KEK'
        if self.has_stepped:
            return 'curr_x=1 curr_v=1 curr_a=2 pole_ang=3 pole_vel=4 timestamp=5 errcode=0'
        else:
            return 'curr_x=0 curr_v=1 curr_a=2 pole_ang=3 pole_vel=4 timestamp=5 errcode=0'


class TestController:
    @staticmethod
    def default_controller(max_steps: int=10):
        return controller.CartPoleController(serial_connection=SerialConnection(), max_steps=max_steps)

    @staticmethod
    def default_action() -> State:
        return 1

    @staticmethod
    def assert_state(state):
        assert isinstance(state, State)
        assert state.cart_position == 0
        assert state.cart_velocity == 1
        assert state.pole_angle == 3
        assert state.pole_velocity == 4


    def test_state(self):
        with mock.patch('serial.Serial', MockSerial):
            c = self.default_controller()
        state = c.state()
        self.assert_state(state)

    def test_reset(self):
        with mock.patch('serial.Serial', MockSerial):
            c = self.default_controller()
        state = c.reset()
        self.assert_state(state)

    def test_step(self):
        with mock.patch('serial.Serial', MockSerial):
            c = self.default_controller()
        state, reward, finish, info = c.step(self.default_action())
        assert isinstance(state, State)
        assert state.cart_position == 1
        assert state.cart_velocity == 1
        assert state.pole_angle == 3
        assert state.pole_velocity == 4
        assert reward == util.reward(state)
        assert finish == False
        assert info == {'step_count': 1}

    def test_step_with_error(self):
        with mock.patch('serial.Serial', functools.partial(MockSerial, with_error=True)):
            c = self.default_controller()
        with pytest.raises(ValueError):
            c.step(self.default_action())
        
    def test_step_with_finish(self):
        with mock.patch('serial.Serial', MockSerial):
            c = self.default_controller(1)
        state, reward, finish, info = c.step(self.default_action())
        assert isinstance(state, State)
        assert state.cart_position == 1
        assert state.cart_velocity == 1
        assert state.pole_angle == 3
        assert state.pole_velocity == 4
        assert reward == util.reward(state)
        assert finish == True
        assert info == {'step_count': 1}
