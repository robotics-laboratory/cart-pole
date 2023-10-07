from cartpole import Error, State
from cartpole import Simulator
from cartpole import log

import time
import torch

# set simulation step as 0.05 seconds
delta = 0.05

# setup logging (look at mcap logs after simulation)
log.setup(log_path='simulation_example.mcap')

# create simulator with default config
cartpole = Simulator()

# reset simulator to initial state
cartpole.reset(state=State(cart_position=0, pole_angle=(2/4 * torch.pi)))
# energy_start = cartpole.evaluate_energy()


# run simulation
for _ in range(1000):
    # use for loggin simulation time instead of real time
    state = cartpole.get_state()

    # log system state and simulator info
    log.publish('/cartpole/state', state, state.stamp)
    log.publish('/cartpole/info', cartpole.get_info(), state.stamp)

    # make simulation step
    cartpole.advance(delta)
    time.sleep(delta)

