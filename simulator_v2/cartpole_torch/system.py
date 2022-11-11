"""
This module contains CartPoleMultiSystem class. The systems data is stored
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

from common import CartPoleBase

from .config import SystemConfiguration
from .discreditizer import Discreditizer
from .learning_context import MultiSystemLearningContext
from .state import MultiSystemState, State


class CartPoleMultiSystem:
    """
    Provides a "functional" interface to evaluate best inputs and new states
    (after applying inputs to given ones).
    The systems data is stored
    in a `MultiSystemLearningContext` and the system itself only performs
    neccessary calculations for multiple CartPole systems at a time.
    """

    @staticmethod
    def eval_transitions(
        context: MultiSystemLearningContext, inputs: DoubleTensor
    ) -> None:
        """
        Calculates new states and writes them in-place.

        Parameters
        ----------
        context : MultiSystemLearningContext
        inputs : DoubleTensor
            A 1xN tensor containing input cart accelerations.
        """
        # Current state
        cur_st = context.batch_state.states
        steps: int = context.config.discretization.integration_step_n
        # dynamics delta time
        d_time: float = 1 / (steps * context.config.discretization.simulation_step_n)

        # Gravitational constant
        grav: float = context.config.parameters.gravity
        pole_len: float = context.config.parameters.pole_length

        def compute_derivative(state: DoubleTensor):
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
            ds1 = compute_derivative(cur_st)  # type: ignore
            ds2 = compute_derivative(cur_st + ds1 * d_time)  # type: ignore
            cur_st += (ds1 + ds2) / 2 * d_time  # type: ignore

    @staticmethod
    def eval_transition_costs(
        context: MultiSystemLearningContext,
        inputs: DoubleTensor,
    ) -> DoubleTensor:
        """
        Evaluates new states cost and action costs.

        Parameters
        ----------
        context : MultiSystemLearningContext
        inputs : DoubleTensor
            The inputs applied

        Returns
        -------
        DoubleTensor
            1xK Tensor containing total cost of applying
            i-th action to i-th state.
        """
        states_cost = context.states_cost_fn(context.batch_state.states)
        inputs_cost = context.inputs_cost_fn(inputs)

        return states_cost + inputs_cost  # type: ignore

    @staticmethod
    def get_best_accelerations(context: MultiSystemLearningContext) -> DoubleTensor:
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

            CartPoleMultiSystem.eval_transitions(context, inputs)
            costs = CartPoleMultiSystem.eval_transition_costs(
                context,
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


@dataclass(init=False)
class CartPoleSystem(CartPoleBase):
    """
    Description:
        Сlass implements a physical simulation of the cart-pole device.
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing the cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    """

    _context: MultiSystemLearningContext
    _current_input: float = 0
    _config: SystemConfiguration = SystemConfiguration()
    _current_time: float = 0

    def _setup_context(
        self,
        state: State,
    ) -> None:
        batch_state = state.as_tensor().reshape(4, -1)
        batch_ms_state = MultiSystemState(batch_state)  # type: ignore
        self._context = MultiSystemLearningContext(
            states_cost_fn=None,  # type: ignore
            inputs_cost_fn=None,  # type: ignore
            config=self._config,
            discreditizer=Discreditizer(self._config),
            batch_state=batch_ms_state,
        )

    def reset(self, config: SystemConfiguration) -> None:
        """
        Resets the device to the initial state.
        The pole is at rest position and cart is centered.
        It must be called at the beginning of any session.
        """
        self._setup_context(State.home())

        self._config = config
        self._current_time = 0

    def reset_to_state(self, config: SystemConfiguration, state: State) -> None:
        self._setup_context(state)

        self._config = config
        self._current_time = 0

    def get_state(self) -> State:
        """
        Returns current device state.
        """
        state_tensor = self._context.batch_state.states.flatten()
        return State.from_collection(state_tensor)  # type: ignore

    def get_info(self) -> dict:
        """
        Returns usefull debug information.
        """
        raise NotImplementedError

    def get_target(self) -> float:
        """
        Returns current target acceleration.
        """
        return self._current_input

    def set_target(self, target: float) -> None:
        """
        Set desired target acceleration.
        """
        self._current_input = target

    def advance(self, delta: float) -> None:
        """
        Advance the dynamic system by delta seconds.
        """
        sim_steps = self._config.discretization.simulation_step_n
        for _ in range(int(delta * sim_steps)):
            CartPoleMultiSystem.eval_transitions(
                context=self._context,
                inputs=torch.full(
                    size=(1,),
                    fill_value=self._current_input,  # type: ignore
                    dtype=torch.float64,
                ),
            )
        self._current_time += delta

    def timestamp(self) -> float:
        """
        Current time.
        """
        return self._current_time
