from state import State

class CartPoleBase:
    def reset(self) -> (State, float):
        '''
        Resets the device to the initial state. The pole is at rest position.
        Returns (initial_state, initial_target).
        '''
        raise NotImplementedError

    def step(self, delta: float) -> None:
        '''
        Make delta time step (delta > 1/240 seconds). In case of control it's a fake call.
        '''
        raise NotImplementedError

    def set_config(self, config) -> None:
        '''
        Reset common config for all envs.
        Specific implementations may have additional settings (e.g sampling parameters in the simulator).
        '''
        raise NotImplementedError

    def get_state(self) -> State:
        '''
        Returns current device state.
        '''
        raise NotImplementedError

    def get_info(self) -> dict:
        '''
        Returns usefull debug information.
        '''
        raise NotImplementedError

    def get_target(self) -> float:
        '''
        Returns current target value.
        '''
        raise NotImplementedError

    def set_target(self, target: float) -> None:
        '''
        Set desired target value.
        '''
        raise NotImplementedError

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError