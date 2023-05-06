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

