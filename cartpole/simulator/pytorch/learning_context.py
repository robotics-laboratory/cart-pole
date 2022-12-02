"""
This module contains a MultiSystemLearningContext which is a container
for all of the data which is used to train a model which
works with batches (multiple systems at a time).
"""

from dataclasses import dataclass, field
from typing import Callable

import torch
from torch import DoubleTensor, IntTensor

from .config import SystemConfiguration
from .discreditizer import Discreditizer
from .state import MultiSystemState

CostFunction = Callable[[DoubleTensor], DoubleTensor]


@dataclass
class MultiSystemLearningContext:
    """
    A container for all the data which is used to train a model which
    works with batches (multiple systems at a time).
    """

    states_cost_fn: CostFunction
    inputs_cost_fn: CostFunction
    config: SystemConfiguration
    batch_state: MultiSystemState
    discreditizer: Discreditizer
    batch_indecies: IntTensor = field(default=torch.empty(0, dtype=torch.int))  # type: ignore

    def update_batch(self, batch_size: int) -> None:
        """
        Generates a new batch and updates the batch multistate.

        Parameters
        ----------
        batch_size : int
            Size of the batch.

        Raises
        ------
        ValueError
            If `batch_size` was less than 1 or greater than
            the total number of states.
        """
        total_states = self.discreditizer.space_size

        if not (0 < batch_size <= total_states):
            raise ValueError("Invalid batch size")

        self.batch_indecies = torch.randint(
            low=0,
            high=total_states,
            size=(batch_size,),  # type: ignore
        )  # type: ignore

        self.batch_state = MultiSystemState.create_from_batch(
            self.discreditizer.all_states,
            self.batch_indecies,  # type: ignore
        )

    @property
    def batch_size(self) -> int:
        """
        Returns the size of the current batch.

        Returns
        -------
        int
        """
        return self.batch_state.size
