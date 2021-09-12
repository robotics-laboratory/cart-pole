from device.wire_interface import WireInterface, DeviceConfig, DeviceTarget
from collector import CollectorProxy
from device import CartPoleDevice

import logging
import time


def main():
    p = CollectorProxy(CartPoleDevice(WireInterface('COM4')))
    
    p.reset(DeviceConfig(debug_led=True))
    p.start()
    a = 0.4    
    p.set_target(-a)
    time.sleep(0.25)

    for _ in range(20):
        p.set_target(a)
        for i in range(4):
            time.sleep(0.1)
            p.get_state()

        a = -a
        p.set_target(a)

        for i in range(4):
            time.sleep(0.1)
            p.get_state()

    assert isinstance(p.cart_pole, CartPoleDevice)

    p.cart_pole.interface.set(DeviceTarget(position=0))

    p.stop()
    p.cart_pole.interface.set(DeviceConfig(debug_led=False))


if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.DEBUG)
    main()
