"""
This module contains Discreditizer class which is responsible for
discreditizing state space (positions, pole angles, velocities and angular
velocities) and action space (cart accelerations).
"""


from dataclasses import dataclass

import torch
from torch import DoubleTensor

from cartpole_torch.config import SystemConfiguration


@dataclass
class Discreditizer:
    """
    Discreditizes state space (positions, pole angles, velocities and angular
    velocities) and action space (cart accelerations) according to the config.

    After discretization, stores the results.
    """

    config: SystemConfiguration

    cart_accelerations: DoubleTensor = None  # type: ignore
    _all_states: DoubleTensor = None  # type: ignore

    def __post_init__(self) -> None:
        """
        Generates all states from configuration.
        """
        positions = torch.linspace(  # type: ignore
            start=-self.config.limits.max_abs_position,
            end=self.config.limits.max_abs_position,
            steps=self.config.discretization.cart_position,
            dtype=torch.float64,
        )
        angles = torch.linspace(  # type: ignore
            start=0.0,
            end=2 * torch.pi,
            steps=self.config.discretization.pole_angle,
            dtype=torch.float64,
        )
        velocities = torch.linspace(  # type: ignore
            start=-self.config.limits.max_abs_velocity,
            end=self.config.limits.max_abs_velocity,
            steps=self.config.discretization.cart_velocity,
            dtype=torch.float64,
        )
        angular_velocities = torch.linspace(  # type: ignore
            start=-self.config.limits.max_abs_angular_velocity,
            end=self.config.limits.max_abs_angular_velocity,
            steps=self.config.discretization.pole_angular_velocity,
            dtype=torch.float64,
        )
        self.cart_accelerations = torch.linspace(  # type: ignore
            start=-self.config.limits.max_abs_acceleration,
            end=self.config.limits.max_abs_acceleration,
            steps=self.config.discretization.cart_acceleration,
            dtype=torch.float64,
        )

        # Generate all possible states tensor
        positions, angles, velocities, angular_velocities = torch.meshgrid(
            [
                positions,
                angles,
                velocities,
                angular_velocities,
            ],
            indexing="ij",
        )

        self._all_states = torch.vstack(  # type: ignore
            [
                positions.flatten(),
                angles.flatten(),
                velocities.flatten(),
                angular_velocities.flatten(),
            ]
        )

    @property
    def space_size(self) -> int:
        """
        Returns the total number of states

        Returns
        -------
        int
        """
        return self._all_states.shape[1]

    @property
    def cart_positions(self) -> DoubleTensor:
        """
        Returns
        -------
        DoubleTensor
            1xN Tensor with cart positions.
        """
        return self._all_states[0]  # type: ignore

    @property
    def pole_angles(self) -> DoubleTensor:
        """
        Returns
        -------
        DoubleTensor
            1xN Tensor with pole angles.
        """
        return self._all_states[1]  # type: ignore

    @property
    def cart_velocities(self) -> DoubleTensor:
        """
        Returns
        -------
        DoubleTensor
            1xN Tensor with cart velocities.
        """
        return self._all_states[2]  # type: ignore

    @property
    def pole_angular_velocities(self) -> DoubleTensor:
        """
        Returns
        -------
        DoubleTensor
            1xN Tensor with pole angular velocities.
        """
        return self._all_states[3]  # type: ignore

    @property
    def all_states(self) -> DoubleTensor:
        """
        Returns
        -------
        DoubleTensor
            4xN Tensor storing cart positions, pole angles, cart velocities
            and pole angular velocities.
        """
        return self._all_states
