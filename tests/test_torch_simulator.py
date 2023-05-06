from cartpole import Error, State, TorchSimulator, TorchSimulatorConfig

import pytest
import math

delta = 0.01

def test_simple_run():
    cartpole = TorchSimulator(config=TorchSimulatorConfig.for_thin_pole())

    init = State(pole_angle=(3/4 * math.pi))
    cartpole.reset(state=init)

    for _ in range(200):
        # check all getters
        stamp = cartpole.timestamp()
        state = cartpole.get_state()
        info = cartpole.get_info()

        # make simulation step
        cartpole.advance(delta)

        # check that state is valid
        assert state.error == Error.NO_ERROR

        # cart is not moving
        assert init.cart_position == pytest.approx(state.cart_position, abs=1e-5)
        assert 0.0 == pytest.approx(state.cart_velocity, abs=1e-5)

        # check that time is increasing
        cartpole.timestamp() > stamp

def test_conservation_of_energy():
    cartpole = TorchSimulator(config=TorchSimulatorConfig.for_thin_pole())

    init = State(pole_angle=(3/4 * math.pi))
    cartpole.reset(state=init)

    energy_start = cartpole.evaluate_energy()

    for _ in range(10000):
        stamp = cartpole.timestamp()
        state = cartpole.get_state()
        info = cartpole.get_info()

        cartpole.advance(delta)

        energy_current = cartpole.evaluate_energy()
        assert energy_start.total == pytest.approx(energy_current.total, abs=1e-5)

def test_fixed_control():
    cartpole = TorchSimulator(config=TorchSimulatorConfig.for_thin_pole())
    cartpole.reset()

    for n in range(200):
        stamp = cartpole.timestamp()
        acc = 0.2 * math.cos(stamp/ 0.1 * math.pi)
        cartpole.set_target(acc)

        cartpole.advance(delta)

def test_position_overflow():
    config = TorchSimulatorConfig.for_thin_pole()
    cartpole = TorchSimulator(config=config)

    cartpole.reset()

    acc = 1.0
    cartpole.set_target(acc)

    timestamp = math.sqrt(2 * config.max_cart_position / acc**2)
    step_n = math.ceil(timestamp / delta)

    for n in range(step_n):
        cartpole.advance(delta)

    state = cartpole.get_state()
    assert state.error == Error.CART_POSITION_OVERFLOW

def test_velocity_overflow():
    config = TorchSimulatorConfig.for_thin_pole(max_cart_position=100.0)
    cartpole = TorchSimulator(config=config)
    cartpole.reset()

    acc = 1.0
    cartpole.set_target(acc)

    timestamp = config.max_cart_velocity / acc
    step_n = math.ceil(timestamp / delta)

    for n in range(step_n):
        cartpole.advance(delta)

    state = cartpole.get_state()
    assert state.error == Error.CART_VELOCITY_OVERFLOW

def test_acceleration_overflow():
    config = TorchSimulatorConfig.for_thin_pole()
    cartpole = TorchSimulator(config=config)
    cartpole.reset()

    acc = config.max_cart_acceleration + 1e-5
    cartpole.set_target(acc)
    state = cartpole.get_state()

    assert state.error == Error.CART_ACCELERATION_OVERFLOW
