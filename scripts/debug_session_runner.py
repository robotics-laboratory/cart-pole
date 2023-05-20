import logging
import time
from pathlib import Path

from cartpole.actors.demo import DemoActor
from cartpole.common.interface import CartPoleBase, Config, Error
from cartpole.common.util import init_logging
from cartpole.device import CartPoleDevice
from cartpole.sessions.actor import Actor
from cartpole.sessions.collector import CollectorProxy

LOGGER = logging.getLogger("debug-session-runner")


def control_loop(device: CartPoleBase, actor: Actor, max_duration: float):
    start = time.perf_counter()
    while time.perf_counter() - start < max_duration:
        # iter_start = time.perf_counter()
        state = device.get_state()
        if state.error != Error.NO_ERROR:
            break
        stamp = time.perf_counter() - start
        target = actor(state, stamp=stamp)
        # logging.info("STAMP: %s", stamp)
        device.set_target(target)
        device.advance()
        # iter_duration = time.perf_counter() - iter_start
        # logging.info(f"Iter duration: {iter_duration:.3f}s, {1/iter_duration:.1f} Hz")


def reset_pole_angle(device: CartPoleDevice):
    # FIXME: Move to firmware
    device.rotations = 0
    device.prev_angle = 0
    device.zero_angle = proxy.get_state().pole_angle
    LOGGER.info(f"Pole angle correction: {device.zero_angle:.5f}")


if __name__ == "__main__":
    from cartpole.common.util import init_logging

    init_logging()

    SESSION_ID = "demo-session"
    SESSION_MAX_DURATION = 60  # Seconds
    OUTPUT_PATH = Path(f"data/sessions/{SESSION_ID}")

    ACTOR_CLASS = DemoActor
    DEVICE_CONFIG = Config(
        max_position=0.4,
        max_velocity=8.0,
        max_acceleration=10.0,
        clamp_velocity=True,
        clamp_acceleration=True,
    )
    ACTOR_CONFIG = dict(
        config=Config(
            max_position=0.35,
            max_velocity=8,
            max_acceleration=10,
            pole_length=0.10,
            pole_mass=0.07
        )
    )

    device = CartPoleDevice()
    proxy = CollectorProxy(
        cart_pole=device,
        actor_class=ACTOR_CLASS,
        actor_config=ACTOR_CONFIG,
    )

    try:
        proxy.reset(DEVICE_CONFIG)
        # exit(0)
        actor = DemoActor(**ACTOR_CONFIG)
        actor.proxy = proxy
        reset_pole_angle(device)  # FIXME
        print("!!!!!!!!\n"*20)
        time.sleep(3)
        control_loop(proxy, actor, max_duration=SESSION_MAX_DURATION)
    except Exception:
        LOGGER.exception("Aborting run due to error")
    finally:
        LOGGER.info("Run finished")
        proxy.set_target(0)
        proxy.close()

    proxy.save(OUTPUT_PATH / "session.json")
