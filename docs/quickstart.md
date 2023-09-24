# Quickstart

## Enviroment
Python is main language of project. So, students may learn control theory and make experiments faster and easier.
Firstly, you need checkout repo and prepare enviroment.

```bash
git clone https://github.com/robotics-laboratory/cart-pole.git
```

We have prepared a container with all dependencies, use it for development and testing
(you need to have [docker](https://docs.docker.com/get-docker/) and [docker-compose](https://docs.docker.com/compose/install/) installed).
Run in root of repo following commands.

```bash
# start container
docker compose up -d

# enter to container
docker exec -it cartpole-{$USER} bash

# run tests to check that everithing is OK
pytest tests
```

Repo folder is mounted to `/cartpole` in container, so you can edit files in your favorite IDE and run scripts in container.
Also there are some environment variables, which may be useful for you:

- `$CONTAINER_NAME` - name of container (default is `cartpole-{$USER}`)
- `$FOXGLOVE_PORT` - port of foxglove server (default is 8765)
- `$DOCKER_RUNTIME` - docker runtime (default is `runc`)

If you want to use your own python environment, you can install all dependencies manually, using [poetry](https://python-poetry.org/).

```bash
# check poetry config
poetry config --list 

# install all depencies to .venv folder
poetry install

# run tests to check that everithing is OK
poetry run pytest tests
```

## Foxglove
For visualization of real time data we use [foxglove studio](https://foxglove.dev/).
We strongly suggest to use our [instance](http://foxglove.robotics-lab.ru), but you may also setupserver with our specific fixes by yourself.
More information [here](https://github.com/robotics-laboratory/foxglove). In foxglove select `Open connection` than `Foxglove WebSocket` and enter `ws://localhost:8765` (use your port) in address field.

## Logging
We have convinient logging system, it may show data in real time and replay saved data in [mcap](https://mcap.dev/) format.

```python
from pydantic import BaseModel

import cartpole.log as log

import random
import time

# all messages must be inherited from BaseModel
class RandMsg(BaseModel):
    dist: str = 'uniform(0, 1)'
    value: float = 0.0

# define log file name
log.setup(log_path='log_example.mcap')

# messages are available in real time in foxglove (websocket mode)
for i in range(20):
    # publish message, timestamp is optional (default is current time)
    log.publish('/random', RandMsg(value=random.random()))
    time.sleep(0.2) # add some delay
```

## Simulation
For development and testing of control algorithms, we provide CartPole simulator, which fully implemntet CartPoleBase [interface](/cartpole/common/interface.py). The simulation is carried out by numerical integration of parameterized dynamic system (more information [here](/docs/cart_pole.pdf)). Also simulator may be used to train ML agents.


```python
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
```
