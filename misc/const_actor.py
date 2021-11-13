from common.interface import State
from sessions.actor import Actor


class ConstantActor(Actor):
    def __init__(self, target_const: float = 0, **kwargs):
        super().__init__(**kwargs)
        self.target_const = target_const

    def __call__(self, state: State) -> float:
        return self.target_const
