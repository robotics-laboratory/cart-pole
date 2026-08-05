"""Simulation / RL types compatible with cart-pole-3 and scripts/cartpole_rl_env.py."""

from __future__ import annotations

import enum
from typing import Any

import numpy as np
from pydantic import BaseModel, ConfigDict, field_validator


class Limits(BaseModel):
    """Constraints of cart state for control and hardware limits."""

    cart_position: float = 0.0
    cart_velocity: float = 0.0
    cart_acceleration: float = 0.0

    def stronger(self, other: "Limits") -> bool:
        return (
            self.cart_position <= other.cart_position
            and self.cart_velocity <= other.cart_velocity
            and self.cart_acceleration <= other.cart_acceleration
        )


class Parameters(BaseModel):
    """CartPole dynamics parameters (simulation only)."""

    gravity: float = 9.81
    friction_coef: float | None = None
    mass_coef: float | None = None

    @staticmethod
    def make(
        mass: float,
        inertia: float,
        length: float,
        friction: float = 0.0,
        gravity: float = 9.81,
    ) -> "Parameters":
        delimiter = mass * length * length + inertia
        return Parameters(
            gravity=gravity,
            friction_coef=friction / delimiter,
            mass_coef=(mass * length) / delimiter,
        )


class Config(BaseModel):
    """Simulation configuration used by CartPoleRLEnv / Simulator."""

    hardware_limit: Limits = Limits()
    control_limit: Limits = Limits()
    parameters: Parameters = Parameters()

    def to_yaml(self) -> str:
        import yaml

        return yaml.dump(self.model_dump(), indent=2)

    def to_yaml_file(self, file_path: str) -> None:
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(self.to_yaml())

    @staticmethod
    def from_yaml(yaml_str: str) -> "Config":
        import yaml

        return Config.model_validate(yaml.load(yaml_str, Loader=yaml.FullLoader))

    @staticmethod
    def from_yaml_file(file_path: str) -> "Config":
        with open(file_path, "r", encoding="utf-8") as f:
            return Config.from_yaml(f.read())


class Error(enum.IntEnum):
    NO_ERROR = 0
    NEED_RESET = 1
    CART_POSITION_OVERFLOW = 2
    CART_VELOCITY_OVERFLOW = 3
    CART_ACCELERATION_OVERFLOW = 4
    HARDWARE = 5
    # Aliases matching the device / legacy Error codes
    X_OVERFLOW = 2
    V_OVERFLOW = 3
    A_OVERFLOW = 4
    MOTOR_STALLED = 5
    ENDSTOP_HIT = 6

    def __bool__(self) -> bool:
        return self != Error.NO_ERROR

    def __repr__(self) -> str:
        return str(self.value)


class State(BaseModel):
    """System state for simulation and reward evaluation."""

    model_config = ConfigDict(use_enum_values=False)

    cart_position: float = 0.0
    cart_velocity: float = 0.0
    cart_acceleration: float = 0.0
    pole_angle: float = 0.0
    pole_angular_velocity: float = 0.0
    stamp: float = 0.0
    error: Error = Error.NO_ERROR

    @field_validator("error", mode="before")
    @classmethod
    def _coerce_error(cls, value: Any) -> Any:
        if isinstance(value, Error):
            return value
        if value is None:
            return Error.NO_ERROR
        return Error(int(value))

    def validate_limits(self, config: Config) -> None:
        if self.error:
            return
        if abs(self.cart_position) > config.hardware_limit.cart_position:
            self.error = Error.CART_POSITION_OVERFLOW
            return
        if abs(self.cart_velocity) > config.hardware_limit.cart_velocity:
            self.error = Error.CART_VELOCITY_OVERFLOW
            return
        if abs(self.cart_acceleration) > config.hardware_limit.cart_acceleration:
            self.error = Error.CART_ACCELERATION_OVERFLOW
            return

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.cart_position,
            self.pole_angle,
            self.cart_velocity,
            self.pole_angular_velocity,
        )

    def numpy4(self) -> np.ndarray:
        return np.array(self.as_tuple(), dtype=np.float32)


class Target(BaseModel):
    """Control command: desired cart acceleration / velocity / position."""

    position: float | None = None
    velocity: float | None = None
    acceleration: float | None = None

    def acceleration_or(self, default: float) -> float:
        return self.acceleration if self.acceleration is not None else default

    def velocity_or(self, default: float) -> float:
        return self.velocity if self.velocity is not None else default

    def validate(self, config: Config) -> None:
        if self.acceleration is not None:
            assert abs(self.acceleration) <= config.control_limit.cart_acceleration
        if self.velocity is not None:
            assert abs(self.velocity) <= config.control_limit.cart_velocity
        if self.position is not None:
            assert abs(self.position) <= config.control_limit.cart_position
