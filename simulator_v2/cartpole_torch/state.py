"""
This module contains 2 important dataclasses:
- `State` which represents the state of one system
- `MultiSystemState` which holds states of multiple systems.
The latter one is usually a batch.
"""


from dataclasses import dataclass
from typing import Collection

from numpy import pi
from torch import DoubleTensor, LongTensor


@dataclass
class State:
    """
    Represents state of the system.

    Fields
    ------
    `cart_position` : float
        Position of a cart along X axis.
        Measured in meters.
        When the cart is in the "home state", the position is `0`.

    `pole_angle` : float
        Angle of the pole with respect to inverted Y axis.
        Measured in radians, always belongs to interval `[0, 2*pi)`.
        When the pole is stable (hanging down), the angle is `0`.

    `cart_velocity` : float
        Velocity of the cart along X axis.
        Measured in m/s (meters per second).

    `angular_velocity` : float
        Angular velocity of the pole in counter-clockwise direction.
        Measured in rad/s (radians per second).
    """

    cart_position: float
    pole_angle: float
    cart_velocity: float
    angular_velocity: float

    def __post_init__(self) -> None:
        self.pole_angle %= 2 * pi

    @staticmethod
    def home() -> "State":
        """
        Returns initial state of the system, where all the fields are set to 0.

        Returns
        -------
        State
            initial state of the system
        """
        return State(0, 0, 0, 0)

    @staticmethod
    def target() -> "State":
        """
        Target (ideal) state of the system.

        Returns
        -------
        State
            Ideal state of the system
        """
        return State(
            cart_position=0,
            pole_angle=pi,
            cart_velocity=0,
            angular_velocity=0,
        )

    def as_tensor(self) -> DoubleTensor:
        """
        Returns current state as a 1x4 tensor

        Returns
        -------
        DoubleTensor
            1x4 Tensor containing `cart_position`,
            `pole_angle`, `cart_velocity` and `angular_velocity`
        """
        return DoubleTensor(
            [
                self.cart_position,
                self.pole_angle,
                self.cart_velocity,
                self.angular_velocity,
            ]
        )

    @staticmethod
    def from_collection(arr: Collection[float]) -> "State":
        """
        Creates a State object from an array of length 4.

        Parameters
        ----------
        arr : Collection[float]
            List/Array/Tensor containing `cart_position`, `pole_angle`,
            `cart_velocity` and `angular_velocity`.

        Returns
        -------
        State
            State created from the collection.

        Raises
        ------
        ValueError
            If length of `arr` is not equal to 4.
        """
        if len(arr) != 4:
            raise ValueError("Length of collection should be 4")
        return State(
            cart_position=arr[0],  # type: ignore
            pole_angle=arr[1],  # type: ignore
            cart_velocity=arr[2],  # type: ignore
            angular_velocity=arr[3],  # type: ignore
        )


@dataclass
class MultiSystemState:
    """
    A class to represent states of multiple systems at a time.
    """

    _state_space: DoubleTensor
    """
    4xN tensor, where N is the number of systems
    - `_state_space[0]` is a 1xN DoubleTensor containing cart positions
    - `_state_space[1]` is a 1xN DoubleTensor containing pole angles
    - `_state_space[2]` is a 1xN DoubleTensor containing cart velocities
    - `_state_space[3]` is a 1xN DoubleTensor containing angular velocities
    """

    @staticmethod
    def home(systems_num: int) -> "MultiSystemState":
        """
        Initializes a MultiSystemState with home states.

        Parameters
        ----------
        systems_num : int
            Number of systems to simulate.

        Returns
        -------
        MultiSystemState
        """
        data = DoubleTensor(size=(4, systems_num))
        home_state = State.home().as_tensor()

        for i in range(systems_num):
            data[:, i] = home_state

        return MultiSystemState(_state_space=data)  # type: ignore

    @staticmethod
    def create_from_batch(
        all_states: DoubleTensor,
        batch: LongTensor,
    ) -> "MultiSystemState":
        """
        Constructs a MultiSystemState from a sample of a state space.

        Parameters
        ----------
        all_states : DoubleTensor
            The state space (4xN DoubleTensor).
        batch : IntTensor
            A vector of integers containing the indexes of states
            we want to simulate.

        Returns
        -------
        MultiSystemState
        """
        state_space = all_states[:, batch]
        return MultiSystemState(_state_space=state_space)  # type: ignore

    @property
    def size(self) -> int:
        """
        Returns the number of systems in current state

        Returns
        -------
        int
            The number of systems in current state
        """
        return self._state_space.shape[1]

    @property
    def states(self) -> DoubleTensor:
        """
        Returns all states

        Returns
        -------
        DoubleTensor
            4xN tensor, where N is the number of systems
            - `state_space[0]` is a 1xN DoubleTensor containing cart positions
            - `state_space[1]` is a 1xN DoubleTensor containing pole angles
            - `state_space[2]` is a 1xN DoubleTensor containing cart velocities
            - `state_space[3]` is a 1xN DoubleTensor containing angular velocities
        """
        return self._state_space

    def set_state_space(self, states: DoubleTensor) -> None:
        """
        Sets `state_space` to given input.

        Args:
        DoubleTensor
            4xN tensor, where N is the number of systems
            - `state_space[0]` is a 1xN DoubleTensor containing cart positions
            - `state_space[1]` is a 1xN DoubleTensor containing pole angles
            - `state_space[2]` is a 1xN DoubleTensor containing cart velocities
            - `state_space[3]` is a 1xN DoubleTensor containing angular velocities
        """
        assert states.shape[0] == 4, "Wrong dimensions specified"
        self._state_space = states
