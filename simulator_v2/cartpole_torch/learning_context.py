"""
This module contains a MultiSystemLearningContext which is a container
for all of the data which is used to train a model which
works with batches (multiple systems at a time).
"""

from dataclasses import dataclass
from typing import Callable

import torch
from torch import DoubleTensor, LongTensor

from cartpole_torch.config import SystemConfiguration
from cartpole_torch.discreditizer import Discreditizer
from cartpole_torch.state import MultiSystemState

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

        if not 0 < batch_size <= total_states:
            raise ValueError("Invalid batch size")

        # Generate 1xK tensor with values from [0, 1)
        rand = torch.rand(batch_size)  # type: ignore

        # Multiply the numbers so they are integers from [0, total_states)
        batch: LongTensor = (rand * total_states).long()  # type: ignore

        self.batch_state = MultiSystemState.from_batch(
            self.discreditizer.all_states,
            batch,
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
