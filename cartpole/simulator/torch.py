from cartpole import Config, Error, State, CartPoleBase

from pydantic import BaseModel
from torch import FloatTensor
from typing import Any

import torch


class TorchSimulatorConfig(Config):
    pole_mass: float = 0.118
    pole_inertia: float = 0.0001
    length: float = 0.15 # distance from joint to center of mass

    gravity: float = 9.81
    time_delta: float = 0.001

class TorchSimulatorInfo(BaseModel):
    step_count: int = 0
    integration_count: int = 0

    class Config:
        @staticmethod
        def schema_extra(schema: Any, model: Any) -> None:
            # make schema lightweight
            properties = schema['properties']           
            for name in properties:
                properties[name].pop('title', None)

class TorchCartPoleSimulator(CartPoleBase):
    def __init__(self, config: TorchSimulatorConfig):
        # super().__init__()
        self.config = config
        self._state = State(error=Error.NEED_RESET)

        self._step_count = 0
        self._integration_count = 0

        l = self.config.length
        m = self.config.pole_mass
        I = self.config.pole_inertia

        self._denominator = l + I / (m * l)


    def _derivative(self, s: FloatTensor, a: float) -> FloatTensor:
        result = torch.empty_like(s)

        theta = s[1]
        g = self.config.gravity
        d = self._denominator

        result[0] = s[2]
        result[1] = s[3]
        result[2] = a
        result[3] = - (a * torch.cos(theta) + g * torch.sin(theta)) / d

        return result

    def reset(self, state: State = State()) -> None:
        self._state = state
        self._count = 0
        self._integration_count = 0

    def get_state(self) -> State:
        return self._state

    def get_info(self) -> TorchSimulatorInfo:
        return TorchSimulatorInfo(
            step_count=self._step_count,
            integration_count=self._integration_count)

    def set_target(self, target: float) -> None:
        if self._state.error:
            return

        self._state.cart_acceleration = target
        self._state.validate(self.config)

    def advance(self, delta: float) -> None:
        if self._state.error:
            return

        s = self._state.torch4()
        a = self._state.cart_acceleration
        dt = self.config.time_delta

        integration_step_n = int(delta / dt)

        for _ in range(integration_step_n):
            ds1 = self._derivative(s, a)
            ds2 = self._derivative(s + ds1 * dt, a)
            s += (ds1 + ds2) / 2 * dt

        self._step_count += 1
        self._integration_count += integration_step_n

        self._state = State(
            cart_position=s[0].item(),
            cart_velocity=s[2].item(),
            pole_angle=s[1].item(),
            pole_angular_velocity=s[3].item(),
            cart_acceleration=a)

        self._state.validate(self.config)

    def timestamp(self) -> float:
        return self.config.time_delta * self._integration_count

    def close(self) -> None:
        pass
