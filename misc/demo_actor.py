from common.interface import State
from sessions.actor import Actor


class ConstantActor(Actor):
    def __init__(self, config, device):
        super().__init__(**kwargs)
        init = State.home()
        self.trajectory = Trajectory(config, init)

        self.balance_control = BalanceLQRControl(config)
        self.trajectory_control = TrajectoryLQRControl(config, trajectory)
        self.begin = None


    def __call__(self, state: State, stamp=None) -> float:
        if stamp < trajectory.duration:
            target = trajectory_control(stamp, state)
        else:
            target = balance_control(state)


