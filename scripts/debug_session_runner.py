import logging
import time
from pathlib import Path

from cartpole.actors.demo import DemoActor
from cartpole.common.interface import CartPoleBase, Config
from cartpole.common.util import init_logging
from cartpole.device import CartPoleDevice
from cartpole.sessions.actor import Actor
from cartpole.sessions.collector import CollectorProxy

LOGGER = logging.getLogger("debug-session-runner")


def control_loop(device: CartPoleBase, actor: Actor, max_duration: float):
    start = time.perf_counter()
    while time.perf_counter() - start < max_duration:
        state = device.get_state()
        stamp = time.perf_counter() - start
        target = actor(state, stamp=stamp)
        logging.info("STAMP: %s", stamp)
        device.set_target(target)
        device.advance()


if __name__ == "__main__":
    from cartpole.common.util import init_logging

    init_logging()

    SESSION_ID = "test-session"
    ACTOR_CLASS = DemoActor
    DEVICE_CONFIG = Config(
        max_position=0.27,
        max_velocity=5,
        max_acceleration=20.0,
        clamp_velocity=True,
        clamp_acceleration=True,
    )
    ACTOR_CONFIG = dict(
        config=Config(
            max_position=0.20,
            max_velocity=4,
            max_acceleration=10.0,
            pole_length=0.28,
        )
    )

    SESSION_MAX_DURATION = 120
    OUTPUT_PATH = Path(f"data/sessions/{SESSION_ID}")
    device = CartPoleDevice()
    actor = ACTOR_CLASS(device_config=DEVICE_CONFIG, **ACTOR_CONFIG)
    proxy = CollectorProxy(
        cart_pole=device,
        actor_class=ACTOR_CLASS,
        actor_config=ACTOR_CONFIG,
    )

    actor.proxy = proxy  # TODO: Refactor

    try:
        proxy.reset(DEVICE_CONFIG)
        time.sleep(5.0)
        control_loop(proxy, actor, max_duration=SESSION_MAX_DURATION)
    except Exception:
        LOGGER.exception("Aborting run due to error")
    finally:
        proxy.close()
        LOGGER.info("Run finished")

    proxy.save(OUTPUT_PATH / "session.json")
