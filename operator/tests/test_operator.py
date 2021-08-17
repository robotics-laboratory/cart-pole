import mock

from cart_pole.common.interface import State, Target, Config
from cart_pole.operator import Operator


EPS = 1e-6

class TestOperator:
    @staticmethod
    def get_operator(max_steps: int = 10):
        return Operator(
            interface=mock.MagicMock(),
            max_steps=max_steps,
        )

    def test_reset(self):
        op = self.get_operator()
        op.reset(Config())
        assert op.interface.reset.called_once

    def test_step(self):
        op = self.get_operator()

        time_delta = 1
        time_passed = 0.5
        time_start = 0
        with mock.patch('time.time', mock.MagicMock(return_value=time_passed)), \
            mock.patch('time.sleep', mock.MagicMock()) as sleep_mock:
            op.get_state_timestamp = time_start
            op.step(time_delta)
            assert sleep_mock.called
            assert abs(time_delta - time_passed - sleep_mock.call_args.args[0]) < EPS
    
    def test_step_without_sleep(self):
        op = self.get_operator()

        with mock.patch('time.time', mock.MagicMock(return_value=100)), \
            mock.patch('time.sleep', mock.MagicMock()) as sleep_mock:
            op.get_state_timestamp = 0
            op.step(1)
            assert sleep_mock.call_count == 0

    def test_get_state(self):
        op = self.get_operator()
        op.get_state()

        assert op.get_state_timestamp is not None
        assert op.interface.get.called_once
        assert len(op.interface.get.call_args.args) == 1
        assert isinstance(op.interface.get.call_args.args[0], State)

    def test_get_target(self):
        op = self.get_operator()
        op.get_target()

        assert op.interface.get.called_once
        assert len(op.interface.get.call_args.args) == 1
        assert isinstance(op.interface.get.call_args.args[0], Target)

    def test_set_target(self):
        op = self.get_operator()
        target = 1
        op.set_target(target)

        assert op.interface.set.called_once
        assert len(op.interface.set.call_args.args) == 1
        arg = op.interface.set.call_args.args[0]
        assert isinstance(arg, Target)
        assert arg.position == target
