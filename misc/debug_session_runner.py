import time
from pathlib import Path
from device import CartPoleDevice, DeviceTarget
from sessions.actor import Actor
from sessions.collector import CollectorProxy
from common.interface import Config, CartPoleBase
from common.util import init_logging
from misc.analyzer._saleae import SaleaeAnalyzer
import logging
# Actor imports
from misc.oscillating_actor import OscillatingActor
from misc.lqr_actor import LinearBalanceControl
from misc.const_actor import ConstantActor

LOGGER = logging.getLogger('debug-session-runner')


def control_loop(device: CartPoleBase, actor: Actor, max_iterations=-1):
    current_iteration = 0
    while current_iteration != max_iterations:
        state = device.get_state()
        target = actor(state)
        device.set_target(target)
        device.make_step()
        current_iteration += 1


if __name__ == '__main__':
    SESSION_ID = 'pb_const_v6'
    OUTPUT_PATH = Path(f'data/sessions/{SESSION_ID}')
    MAX_ITERATIONS = 500

    ### Oscillating actor session
    # ACTOR_CLASS = OscillatingActor
    # ACTOR_CONFIG = {'acceleration': 1.0, 'max_position': 0.05}
    # DEVICE_CONFIG = Config(max_position=0.25, max_velocity=2.0, max_acceleration=10.0)

    ### LQR session
    # ACTOR_CLASS = LinearBalanceControl
    # ACTOR_CONFIG = {'gravity': 9.8, 'pole_length': 0.3, 'eps': 0.2, 'countdown': 2}
    # DEVICE_CONFIG = Config(
    #     max_position=0.25,
    #     max_velocity=1.0,
    #     max_acceleration=5.0,
    #     clamp_velocity=True,
    #     clamp_acceleration=True,
    # )

    ### Const session
    ACTOR_CLASS = ConstantActor
    ACTOR_CONFIG = {}
    DEVICE_CONFIG = Config(max_position=0.25, max_velocity=2.0, max_acceleration=10.0)

    init_logging()
    device = CartPoleDevice()
    # analyzer = SaleaeAnalyzer()
    actor = ACTOR_CLASS(device_config=DEVICE_CONFIG, **ACTOR_CONFIG)
    proxy = CollectorProxy(
        cart_pole=device,
        actor_class=ACTOR_CLASS,
        actor_config=ACTOR_CONFIG,
        # reset_callbacks=[analyzer.start],
        # close_callbacks=[analyzer.stop],
    )

    try:
        proxy.reset(DEVICE_CONFIG)
        LOGGER.info(">>>")
        device.interface.set(DeviceTarget(position=0.05))
        LOGGER.info("<<<")
        control_loop(proxy, actor, max_iterations=MAX_ITERATIONS)
    except Exception:
        LOGGER.exception('Aborting run due to error')
    finally:
        proxy.close()
        LOGGER.info('Run finished')

    proxy.save(OUTPUT_PATH / 'session.json')
    # analyzer.save(OUTPUT_PATH)
