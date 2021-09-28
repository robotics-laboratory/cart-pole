from interface import State


class Actor:
    '''Base class for cartpole control algorithms'''

    def __init__(self, **_):
        '''Implement this to use extra params (kwargs) for your algorithm'''
        pass

    def __call__(self, state: State) -> float:
        raise NotImplementedError
