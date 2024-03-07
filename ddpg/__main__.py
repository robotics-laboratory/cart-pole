import sys
sys.path.append("/Users/severovv/hse/cart-pole/cart-pole")

from cartpole.simulator import Simulator, State, Error, Target
from cartpole import log

from trainer import Trainer, Config, compute_reward, Scalar

from itertools import count

import tqdm
import torch

config = Config(
    state_dim = 5,
    action_dim = 1,
    max_action = 1,
    discount = 0.99,
    memory_size = int(1e6),
    device_name = 'cpu',
    batch_size = 1024,
    actor_lr = 1e-4,
    critic_lr = 2e-4,
    tau = 0.005
)

delta = 0.005
log.setup(log_path='training.mcap', level=log.Level.DEBUG)


cartpole = Simulator()
trainer = Trainer(log, config)

total = 0
state = State()
for episode in range(30):
    cartpole.reset
    state = State(stamp=state.stamp)
    cartpole.reset(state=state)
    print(state.stamp)

    for _ in tqdm.tqdm(range(1000000)):
        total += 1
        prev_state = state

        cartpole.advance(delta)
        state = cartpole.get_state()
        reward = compute_reward(state)

        if total > 3000:
            action = trainer.select_action(state, sigma=0.2, train=True).item()
        else:
            action = torch.normal(0.0, 0.7, (1,)).item()

        trainer.memory.add(prev_state, state, action, reward.value)
        cartpole.set_target(Target(acceleration=action))

        trainer.learn(total=total)

        log.publish('/cartpole/state', state, state.stamp)
        log.publish('/cartpole/reward', reward, state.stamp)

        if state.error != Error.NO_ERROR:
            break