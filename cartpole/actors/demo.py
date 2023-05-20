from cartpole.common.interface import State
from cartpole.sessions.actor import Actor
from cartpole.control import BalanceLQRControl, TrajectoryLQRControl, Trajectory
from logging import getLogger
import math

logger = getLogger(__file__)

class DemoActor(Actor):
    def __init__(self, config, **kwargs):
        super().__init__(**kwargs)
        logger.info("Calculating trajectory...")
        init = State.home()
        self.trajectory = Trajectory(config, init)

        self.balance_control = BalanceLQRControl(config)
        self.trajectory_control = TrajectoryLQRControl(config, self.trajectory)
        self.begin = None
        self.proxy = None
        self.mode = "trajectory"
        logger.info("Trajectory ready")

    def __call__(self, state: State, stamp=None) -> float:
        self.save_expected_state(stamp)
        old_mode = self.mode
        if abs(state.pole_angle - math.pi) < 0.1:
            self.mode = "balance"
        if self.mode == "trajectory":
            target = self.trajectory_control(max(0, stamp), state)
        if self.mode == "balance":
            target = self.balance_control(state)
        if old_mode != self.mode:
            print(f"NEW MODE: {self.mode} \n" * 5)
        return float(target)

    def save_expected_state(self, stamp):
        state_expected, target_expected = self.trajectory(stamp)
        self.proxy.push_foxglove_data(
            {
                'exp_cart_position': float(state_expected.cart_position),
                'exp_cart_velocity': float(state_expected.cart_velocity),
                'exp_pole_angle': float(state_expected.pole_angle),
                'exp_pole_angular_velocity': float(state_expected.pole_angular_velocity),
                'exp_cart_acceleration': float(target_expected),
            }
        )
        # self.proxy._add_value('expected.cart_position', ts, float(state_expected.cart_position))
        # self.proxy._add_value('expected.cart_velocity', ts, float(state_expected.cart_velocity))
        # self.proxy._add_value('expected.pole_angle', ts, float(state_expected.pole_angle))
        # self.proxy._add_value('expected.pole_angular_velocity', ts, float(state_expected.pole_angular_velocity))
        # self.proxy._add_value('expected.target', ts, float(target_expected))
