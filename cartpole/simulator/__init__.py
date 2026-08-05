"""RK4 CartPole simulator compatible with scripts/cartpole_rl_env.py."""

from __future__ import annotations

import numpy as np
from pydantic import BaseModel

from cartpole.common.rl_types import (
    Config,
    Error,
    Limits,
    Parameters,
    State,
    Target,
)


def _sgn(x: float) -> int:
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def _snap_zero(x: float, eps: float) -> float:
    return 0.0 if abs(x) < eps else x


class SimulatorInfo(BaseModel):
    step_count: int = 0
    integration_count: int = 0


class Simulator:
    """CartPole simulator using RK4 integration (cart-pole-3 compatible)."""

    def __init__(self, integration_step: float = 0.001):
        self._config = Config(
            hardware_limit=Limits(
                cart_position=0.5,
                cart_velocity=2.0,
                cart_acceleration=5.0,
            ),
            control_limit=Limits(
                cart_position=0.15,
                cart_velocity=2.0,
                cart_acceleration=3.0,
            ),
            parameters=Parameters(
                gravity=9.81,
                friction_coef=0.0,
                mass_coef=0.3,
            ),
        )
        self._state = State(error=Error.NEED_RESET)
        self._target = Target(velocity=0.0)
        self._integration_step = float(integration_step)
        self._integration_count = 0
        self._step_count = 0

    def get_config(self) -> Config:
        return self._config

    def set_config(self, config: Config) -> None:
        assert config.control_limit.stronger(config.hardware_limit)
        assert config.parameters.friction_coef is not None
        assert config.parameters.friction_coef >= 0
        assert config.parameters.mass_coef is not None
        assert config.parameters.mass_coef > 0
        self._config = config

    def _eval_acceleration_by_velocity(
        self, velocity: float, eps: float = 1e-6
    ) -> float:
        err = velocity - self._state.cart_velocity
        a = self._target.acceleration_or(self._config.control_limit.cart_acceleration)
        if abs(err) < a * self._integration_step:
            return _snap_zero(err / self._integration_step, eps)
        return _sgn(err) * a

    def _eval_cart_acceleration(self, eps: float = 1e-6) -> float:
        if self._target.position is not None:
            a = self._target.acceleration_or(
                self._config.control_limit.cart_acceleration
            )
            v = self._target.velocity_or(self._config.control_limit.cart_velocity)
            err = self._target.position - self._state.cart_position
            if abs(err) < 1e-3:
                return self._eval_acceleration_by_velocity(0.0)
            if _sgn(err) != _sgn(self._state.cart_velocity):
                return self._eval_acceleration_by_velocity(_sgn(err) * v)
            a_brake = self._state.cart_velocity**2 / (2 * abs(err))
            if a_brake >= a:
                return self._eval_acceleration_by_velocity(0.0)
            return self._eval_acceleration_by_velocity(_sgn(err) * v)

        if self._target.velocity is not None:
            return self._eval_acceleration_by_velocity(self._target.velocity, eps)

        if self._target.acceleration is not None:
            return float(self._target.acceleration)

        raise ValueError("At least one target field is required")

    def _derivative(self, s: np.ndarray, a: float) -> np.ndarray:
        result = np.zeros(4, dtype=np.float64)
        b = float(self._config.parameters.friction_coef or 0.0)
        k = float(self._config.parameters.mass_coef or 0.0)
        g = float(self._config.parameters.gravity)
        result[0] = s[2]
        result[1] = s[3]
        result[2] = a
        result[3] = -b * s[3] - k * (a * np.cos(s[1]) + g * np.sin(s[1]))
        return result

    def reset(self, state: State | None = None) -> None:
        self._state = state if state is not None else State()
        self._target = Target(velocity=0.0)
        self._integration_count = 0
        self._step_count = 0

    def get_state(self) -> State:
        return self._state

    def get_info(self) -> SimulatorInfo:
        return SimulatorInfo(
            step_count=self._step_count,
            integration_count=self._integration_count,
        )

    def set_target(self, target: Target) -> State:
        self._target = target
        if not self._state.error:
            self._state.cart_acceleration = self._eval_cart_acceleration()
            self._state.validate_limits(self._config)
        return self.get_state()

    def advance(self, delta: float) -> None:
        if self._state.error:
            return

        s = self._state.numpy4().astype(np.float64)
        h = self._integration_step
        h_2 = h / 2.0
        integration_step_n = int(delta / h)
        a = 0.0

        for _ in range(integration_step_n):
            a = self._eval_cart_acceleration()
            k1 = self._derivative(s, a)
            k2 = self._derivative(s + k1 * h_2, a)
            k3 = self._derivative(s + k2 * h_2, a)
            k4 = self._derivative(s + k3 * h, a)
            s += (k1 + 2 * k2 + 2 * k3 + k4) * h / 6.0
            self._integration_count += 1

        self._state = State(
            cart_position=float(s[0]),
            cart_velocity=float(s[2]),
            cart_acceleration=float(a),
            pole_angle=float(s[1]),
            pole_angular_velocity=float(s[3]),
            stamp=self._integration_step * self._integration_count,
        )
        self._state.validate_limits(self._config)
        self._step_count += 1

    def close(self) -> None:
        pass


__all__ = ["Simulator", "SimulatorInfo"]
