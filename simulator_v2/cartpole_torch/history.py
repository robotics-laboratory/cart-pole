"""
This module contains 3 classes related to keeping record of systems states:
- `HistoryTensorFields` is a simple enum which assigns
    human-readable names to tensor rows.
- `HistoryEntry` which represents state of the system at a certain
    point in time before applying a specified input.
- `SystemHistory` which stores `HistoryEntries` and provides convenient
    methods to explore data.
"""


from dataclasses import dataclass
from enum import IntEnum, auto

import torch
from torch import DoubleTensor, Tensor, cos

from cartpole_torch.config import SystemParameters
from cartpole_torch.state import State


class HistoryTensorFields(IntEnum):
    """
    Maps Tensor rows into human-readable constants and vice-versa.
    """

    TIMESTAMP = 0
    INPUT = auto()
    CART_POSITION = auto()
    POLE_ANGLE = auto()
    CART_VELOCITY = auto()
    POLE_ANGULAR_VELOCITY = auto()


@dataclass
class HistoryEntry:
    """
    Represents state of the system at a certain point in time before
    applying a specified input.

    Fields
    ------
    `timestamp` : float
        Time (in seconds) since the start of the simulation.

    `current_input` : float
        Input to the system during current step.
        Measured in m/s^2 (meters per second squared).

    `state` : State
        State of the system at the end of the current step.
    """

    timestamp: float
    current_input: float
    state: State

    def as_tensor(self) -> DoubleTensor:
        """
        Represent history entry as a 1x6 tensor.

        Returns
        -------
        DoubleTensor
            Tensor of length 6 containing the following columns:
            - `timestamp` - time since start of the simulation (in seconds)
            - `input` - input to the system (float, m/s^2)
            - `position` - position of the cart (float, m)
            - `angle` - angle of the pole (float, rad)
            - `velocity` - velocity of the cart (float, m/s)
            - `angular_velocity` - angular velocity of the pole (float, rad/s)
        """

        pos, angle, velocity, ang_velocity = self.state.as_tensor()
        return DoubleTensor(
            [
                self.timestamp,
                self.current_input,
                pos,
                angle,
                velocity,
                ang_velocity,
            ]
        )

    @staticmethod
    def from_tensor(data: DoubleTensor) -> "HistoryEntry":
        """
        Creates a History Entry from tensor with its data.

        Parameters
        ----------
        data : DoubleTensor
            Tensor of length 6 containing the following columns:
            - `timestamp` - time since start of the simulation (in seconds)
            - `input` - input to the system (float, m/s^2)
            - `position` - position of the cart (float, m)
            - `angle` - angle of the pole (float, rad)
            - `velocity` - velocity of the cart (float, m/s)
            - `angular_velocity` - angular velocity of the pole (float, rad/s)

        Returns
        -------
        HistoryEntry
        """
        timestamp = float(data[HistoryTensorFields.TIMESTAMP])
        current_input = float(data[HistoryTensorFields.INPUT])

        state = State.from_collection(
            [
                float(data[HistoryTensorFields.CART_POSITION]),
                float(data[HistoryTensorFields.POLE_ANGLE]),
                float(data[HistoryTensorFields.CART_VELOCITY]),
                float(data[HistoryTensorFields.POLE_ANGULAR_VELOCITY]),
            ]
        )

        return HistoryEntry(timestamp, current_input, state)


@dataclass
class SystemHistory:
    """
    A smart container for history entries which provides convenient
    methods to explore data.

    Allows to iterate over HistoryEntries while still being fast
    when adding new entries.
    """

    _history: DoubleTensor = DoubleTensor()
    __iter_index = -1

    def add_entry(
        self,
        timestamp: float,
        current_input: float,
        state: State,
    ) -> None:
        """
        Adds an entry to history

        Parameters
        ----------
        `timestamp` : float
            Time (in seconds) since the start of the simulation.

        `current_input` : float
            Input to the system during current step.
            Measured in m/s^2 (meters per second squared).

        `state` : State
            Current state of the system (after the step).
        """
        entry = HistoryEntry(timestamp, current_input, state)
        entry_t = entry.as_tensor().reshape(1, -1)
        self._history = torch.cat((self._history, entry_t))  # type: ignore

    def as_tensor(self) -> DoubleTensor:
        """
        Converts history to a Nx6 tensor

        Returns
        -------
        Tensor
            Nx6 tensor containing the following columns:
            - `timestamp` - time since start of the simulation (in seconds)
            - `input` - input to the system (float, m/s^2)
            - `cart_position` - position of the cart (float, m)
            - `pole_angle` - angle of the pole (float, rad)
            - `cart_velocity` - velocity of the cart (float, m/s)
            - `pole_angular_velocity` - angular velocity of the
            pole (float, rad/s)
        """
        return self._history

    def timestamps(self) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with timestamps.
        """
        return self._history[:, HistoryTensorFields.TIMESTAMP]

    def inputs(self) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with inputs to the system.
        """
        return self._history[:, HistoryTensorFields.INPUT]

    def cart_positions(self) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with positions of the cart.
        """
        return self._history[:, HistoryTensorFields.CART_POSITION]

    def pole_angles(self) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with angles of the pole.
        """
        return self._history[:, HistoryTensorFields.POLE_ANGLE]

    def cart_velocities(self) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with velocities of the cart.
        """
        return self._history[:, HistoryTensorFields.CART_VELOCITY]

    def pole_angular_velocities(self) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with angles of the pole.
        """
        return self._history[:, HistoryTensorFields.POLE_ANGULAR_VELOCITY]

    @property
    def size(self) -> int:
        """
        Returns the number of records in history.
        """
        return self._history.shape[0]

    def total_energies(self, config: SystemParameters) -> Tensor:
        """
        Returns
        -------
        Tensor
            1xN Tensor with total energies at each step.
        """
        kin_cart: DoubleTensor = (
            config.cart_mass * (self.cart_velocities() ** 2) / 2  # type: ignore
        )

        pot_pole = (
            config.pole_mass
            * config.gravity
            * config.pole_length
            / 2
            * (1 - cos(self.pole_angles()))
        )

        velocities = self.cart_velocities()
        angular_velocities = self.pole_angular_velocities()
        kin_pole = (config.pole_mass / 2) * (
            velocities**2
            + ((config.pole_length**2) * (angular_velocities**2)) / 3
            + (
                config.pole_length
                * velocities
                * angular_velocities
                * cos(self.pole_angles())
            )
        )

        # cart does not have a potential energy since it
        # does not travel vertically
        return kin_cart + kin_pole + pot_pole

    def __iter__(self) -> "SystemHistory":
        self.__iter_index = -1
        return self

    def __next__(self) -> HistoryEntry:
        self.__iter_index += 1

        if self.__iter_index == self.size:
            raise StopIteration

        data = self._history[self.__iter_index]
        return HistoryEntry.from_tensor(data)  # type: ignore

    def get_entries(self) -> list[HistoryEntry]:
        """
        Returns history as a list of History Entries (costy!).

        Returns
        -------
        list[HistoryEntry]
        """
        return [HistoryEntry.from_tensor(row) for row in self._history]  # type: ignore

    # TODO: add actual smart methods
    # like plotting / generating animations / smth else
