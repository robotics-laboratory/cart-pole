from cartpole import Error, State
from cartpole import TorchSimulator, TorchSimulatorConfig
from cartpole import log

import time
import torch

# set simulation step as 0.05 seconds
delta = 0.05

# setup logging (look at mcap logs after simulation)
log.setup(log_path='simulation_example.mcap')

# create simulator with default config
config = TorchSimulatorConfig.for_thin_pole()
cartpole = TorchSimulator(config=config)

# reset simulator to initial state
cartpole.reset(state=State(cart_position=0, pole_angle=(2/4 * torch.pi)))
energy_start = cartpole.evaluate_energy()


# run simulation
for _ in range(1000):
    # use for loggin simulation time instead of real time
    stamp = cartpole.timestamp()

    # log system state and simulator info
    log.publish('/cartpole/state', cartpole.get_state(), stamp)
    log.publish('/cartpole/info', cartpole.get_info(), stamp)

    # make simulation step
    cartpole.advance(delta)
    time.sleep(delta)

