from cartpole.simulator import Simulator, State, Error, Target
from cartpole import log

from ddpg.trainer import Trainer, Config, compute_reward, Scalar

from itertools import count

import tqdm
import torch

config = Config(
    state_dim = 5,
    action_dim = 1,
    max_action = 0.7,
    discount = 0.99,
    memory_size = int(1e6),
    device_name = 'cpu',
    batch_size = 256,
    actor_lr = 1e-3,
    critic_lr = 1e-3,
    tau = 0.05,
)

delta = 0.1
log.setup(log_path='training.mcap', level=log.Level.DEBUG)


cartpole = Simulator()
trainer = Trainer(log, config)

total = 0
for episode in count():
    state = State(cart_position=0.1, pole_angle=0.1)
    cartpole.reset(state=state)

    for _ in tqdm.tqdm(range(10000)):
        total += 1
        prev_state = state

        cartpole.advance(delta)
        state = cartpole.get_state()
        reward = compute_reward(state)

        if total > 1000:
            action = trainer.select_action(state, sigma=0.1, train=True).item()
        else:
            action = torch.normal(0.0, 0.5, (1,)).item()

        trainer.memory.add(prev_state, state, action, reward.value)
        cartpole.set_target(Target(acceleration=action))

        trainer.learn(total=total)

        log.publish('/cartpole/state', state, state.stamp)
        log.publish('/cartpole/reward', reward, state.stamp)

        if state.error != Error.NO_ERROR:
            break