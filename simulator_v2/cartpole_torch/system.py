"""
This module contains 2 classes which both represent CartPole systems.
They are responsible for simulating the physics, although they have some
differences:
- `CartPoleSystem` stores all the information about one system and
    evaluates the derivatives to get new state. It cannot calculate best
    inputs (for now).
- `CartPoleMultiSystem` has a different approach. The systems data is stored
    in a `MultiSystemLearningContext` and the system itself only performs
    neccessary calculations for multiple CartPole systems at a time.
    It provides a "functional" interface which allows us to change contexts
    on fly if needed.

    It supports evaluating best inputs for a given set of states and also
    evaluating new states (after inputs are applied).
"""


from dataclasses import dataclass

import torch
from torch import DoubleTensor, cos, sin

from cartpole_torch.config import SystemConfiguration
from cartpole_torch.history import SystemHistory
from cartpole_torch.learning_context import MultiSystemLearningContext
from cartpole_torch.state import State

# FIXME: CartPoleSystem and CartPoleMultiSystem are not interchangable.
# Think of a way to create some kind of a base class
# Or simply say that CartPoleSystem is a MultiSystem with 1 state.


@dataclass
class CartPoleSystem:
    """
    Defines the system as a whole

    Fields
    ------
    `config` : SystemConfiguration
        Physical configuration of the system.

    `current_state` : State
        Current state of the system.
        Changes over time.

    `target_state` : State
        State we want the system to be in.
        Should remain constant.

    `simulation_time` : float
        Time (in seconds) since the start of the simulation.
    """

    config: SystemConfiguration = SystemConfiguration()
    current_state: State = State.home()
    # TODO: remove field and use State.target() to ensure immutability
    # TODO: think of cases when the field should be mutable
    target_state: State = State.target()
    simulation_time: float = 0.0
    history: SystemHistory = SystemHistory()
    # Pre-allocated tensors which are often used
    # ds1 and ds2 are used for derivative computation
    __ds1 = DoubleTensor([0, 0, 0, 0])  # delta state 1
    __ds2 = DoubleTensor([0, 0, 0, 0])  # delta state 2

    def advance_to(self, target_time: float, best_input: float) -> None:
        """
        Advances the system to a given moment in time.

        Parameters
        ----------
        `target_time` : float
            New time of the system.
            Should be greater than `simulation_time`.
        `best_input` : float
            Input to the system (in m/s^2).
            Will be removed.

        Raises
        ------
        ValueError
            If target time is smaller than `simulation_time`.
        """
        if target_time < self.simulation_time:
            raise ValueError("Target time should be greater than current time")
        # FIXME: Remove best input from args and calculate it each time
        while self.simulation_time < target_time:
            self.advance_one_step(best_input)

    def advance_one_step(self, best_input: float) -> None:
        """
        Advances the system one step further.
        One step equals `config.input_timestep` seconds.
        """
        # Current state
        cur_st: DoubleTensor = self.current_state.as_tensor()
        steps: int = self.config.discretization.dynamics_steps_per_input
        # Delta time
        d_time: float = 1 / (steps * self.config.discretization.input_frequency)

        grav: float = self.config.parameters.gravity  # Gravitational constant
        pole_len: float = self.config.parameters.pole_length

        def __compute_derivative(
            state: DoubleTensor,
            delta_state: DoubleTensor,
        ) -> None:
            ang = state[1]  # 1x1 Tensor with angle
            delta_state[0] = state[2]
            delta_state[1] = state[3]
            delta_state[2] = best_input
            delta_state[3] = (
                (-3 / 2) * (best_input * cos(ang) + grav * sin(ang)) / pole_len
            )

        for _ in range(steps):
            # Evaluate derivatives
            __compute_derivative(
                state=cur_st,
                delta_state=self.__ds1,
            )
            __compute_derivative(
                state=cur_st + self.__ds1 * d_time,  # type: ignore
                delta_state=self.__ds2,
            )
            cur_st += (self.__ds1 + self.__ds2) / 2 * d_time  # type: ignore

        self.history.add_entry(
            timestamp=self.simulation_time,
            current_input=best_input,
            state=self.current_state,
        )
        self.current_state = State.from_collection(cur_st)  # type: ignore
        self.simulation_time += 1 / self.config.discretization.input_frequency


class CartPoleMultiSystem:
    """
    Provides a "functional" interface to evaluate best inputs and new states
    (after applying inputs to given ones).
    The systems data is stored
    in a `MultiSystemLearningContext` and the system itself only performs
    neccessary calculations for multiple CartPole systems at a time.
    """

    @staticmethod
    def get_new_states(
        context: MultiSystemLearningContext, inputs: DoubleTensor
    ) -> DoubleTensor:
        """
        Reterns positions after applying `inputs`

        Parameters
        ----------
        context : MultiSystemLearningContext
        inputs : DoubleTensor
            A 1xN tensor containing input cart accelerations.

        Returns
        -------
        DoubleTensor
            A 4xN Tensor containing N states after an input tick.
            By default input tick is
            `1 / context.config.discretization.input_time`
        """
        # Current state
        cur_st = context.batch_state.states
        cur_st: DoubleTensor = torch.clone(cur_st)  # type: ignore
        steps: int = context.config.discretization.dynamics_steps_per_input
        # dynamics delta time
        d_time: float = 1 / (steps * context.config.discretization.input_frequency)

        # Gravitational constant
        grav: float = context.config.parameters.gravity
        pole_len: float = context.config.parameters.pole_length

        def __compute_derivative(state: DoubleTensor):
            # FIXME : Use 2 pre-allocated arrays instead of creating new ones
            ang = state[1]
            return torch.vstack(
                (
                    state[2],
                    state[3],
                    inputs,
                    -1.5 / pole_len * (inputs * cos(ang) + grav * sin(ang)),
                )
            )

        for _ in range(steps):
            # Evaluate derivatives
            ds1 = __compute_derivative(cur_st)
            ds2 = __compute_derivative(cur_st + ds1 * d_time)  # type: ignore
            cur_st += (ds1 + ds2) / 2 * d_time  # type: ignore

        return cur_st

    @staticmethod
    def eval_transition_costs(
        context: MultiSystemLearningContext,
        new_states: DoubleTensor,
        inputs: DoubleTensor,
    ) -> DoubleTensor:
        """
        Evaluates new states cost and action costs.

        Parameters
        ----------
        context : MultiSystemLearningContext
        new_states : DoubleTensor
            States after applying inputs (after transition).
        inputs : DoubleTensor
            The inputs applied

        Returns
        -------
        DoubleTensor
            1xK Tensor containing total cost of applying
            i-th action to i-th state.
        """
        states_cost = context.states_cost_fn(new_states)
        inputs_cost = context.inputs_cost_fn(inputs)

        return states_cost + inputs_cost  # type: ignore

    @staticmethod
    def get_best_accelerations(
        context: MultiSystemLearningContext,
    ) -> DoubleTensor:
        """
        Returns best input for each state.

        Parameters
        ----------
        context : MultiSystemLearningContext
        Returns
        -------
        DoubleTensor
            1xK Tensor with best inputs for all the states.
        """
        best_costs: DoubleTensor = torch.full(
            size=(context.batch_size,),
            fill_value=torch.inf,
            dtype=torch.float64,
        )  # type: ignore
        best_inputs: DoubleTensor = torch.zeros(
            size=(context.batch_size,),
            dtype=torch.float64,
        )  # type: ignore

        for acc in context.discreditizer.cart_accelerations:
            inputs: DoubleTensor = torch.full(
                size=(context.batch_size,),
                fill_value=acc,  # type: ignore
                dtype=torch.float64,
            )  # type: ignore

            new_states = CartPoleMultiSystem.get_new_states(context, inputs)
            costs = CartPoleMultiSystem.eval_transition_costs(
                context,
                new_states,
                inputs,
            )

            # Define a mask which says if i-th cost was better than current one
            rng = range(context.batch_size)

            better_mask = [costs[i] < best_costs[i] for i in rng]

            # update minimum costs
            best_costs = torch.minimum(best_costs, costs)  # type: ignore
            # update best inputs
            best_inputs[better_mask] = acc

        return best_inputs
