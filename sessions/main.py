from device.wire_interface import WireInterface, DeviceConfig
from sessions.runner import Runner
from sessions.actor import OscillatingActor
from device import CartPoleDevice

import logging


def main():
    device = CartPoleDevice()
    config = DeviceConfig(max_velocity=0.25, max_acceleration=1)
    runner = Runner(device, config, OscillatingActor, {
        'device_config': config,
        'acceleration': 0.5, 
        'max_position': 0.1
    })

    runner.start_server()

    runner.run(1000)

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.DEBUG)
    main()
