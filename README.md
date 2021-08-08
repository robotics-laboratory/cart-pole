# Cart pole

## Interface
We declare a common interface both for simulator and control.
It allows us to easily train model and make inference on device,
use transfer learning, generate real training samples and etc.

### State
**Field**     | **Unit** | **Description**
------------- | -------- | ---------------
cart_positon  | m        | Position of the cart along the guide axis. The middle is a reference point.
cart_velocity | m/s      | Instantaneous linear speed of the cart.
pole_angle    | rad      | Angle of pole (ccw). The he lowest position is the reference point. 
pole_velocity | rad/s    | Instantaneous angular velocity of the pole.
pole_velocity | Enum     | Error codes describing the state of the cart-pole.

**Code**              | **Int** | **Description**
--------------------- | ------- | ---------------
NO_ERROR              | 0       | Position of the cart along the guide axis. The middle is a reference point.
NO_CONTROL            | 1       | Environment loose control (use soft or hard reset).
NOT_INITIALIZED       | 2       | Neeed initial device reset.
INVALID_CART_POSITION | 3       | The maximum allowed position is achieved.
INVALID_CART_VELOCITY | 4       | The maximum speed is exceeded.

### Target space
Before usage, you should set config, specifying the type of control:
- POSTION_CONTROL
- VELOCITY_CONTROL
- ACCELERATION_CONTROL

The target space is always some interval (defined with acceptable limits)
so control target is encoded as float scalar.

### API
```
class CartPoleBase:
    def reset(self) -> (State, float):
        '''
        Resets the device to the initial state. The pole is at rest position.
        Returns (initial_state, initial_target).
        '''

    def step(self, delta: float) -> None:
        '''
        Make delta time step (delta > 1/240 seconds). In case of control it's a fake call.
        '''

    def set_config(self, config) -> None:
        '''
        Reset common config for all envs.
        Specific implementations may have additional settings (e.g sampling parameters in the simulator).
        '''

    def get_state(self) -> State:
        '''
        Returns current device state.
        '''

    def get_info(self) -> dict:
        '''
        Returns usefull debug information.
        '''

    def get_target(self) -> float:
        '''
        Returns current target value.
        '''

    def set_target(self, target: float) -> None:
        '''
        Set desired target value.
        '''

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
```

## Control
Comming soon (short description and link to protocol)...

## Simulator
Comming soon (more about simulation and sampling of model parameters)...