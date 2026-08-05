from cartpole.common.interface import Config, Error, State, CartPoleBase
from cartpole.common.rl_types import (
    Config as SimConfig,
    Limits,
    Parameters,
    State as SimState,
    Target,
)

__all__ = [
    "CartPoleBase",
    "Config",
    "Error",
    "Limits",
    "Parameters",
    "SimConfig",
    "SimState",
    "State",
    "Target",
    "generate_pyplot_animation",
]


def __getattr__(name: str):
    if name == "generate_pyplot_animation":
        from cartpole.common.view import generate_pyplot_animation

        return generate_pyplot_animation
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
