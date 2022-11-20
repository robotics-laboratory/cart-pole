# How to record sessions

```python
from cartpole.device.wire_interface import WireInterface, DeviceConfig, DeviceTarget
from collector import CollectorProxy
from cartpole.device import CartPoleDevice
import logging


def main():
    wire = WireInterface('COM4')
    cartpole = CartPoleDevice(wire, target_key='acceleration')
    p = CollectorProxy(cartpole)
    p.reset(DeviceConfig(debug_led=True, max_velocity=1.0, max_acceleration=3))
    p.start()
    
    # CONTROL THE DEVICE HERE
    
    p.cart_pole.interface.set(DeviceTarget(position=0))
    p.stop()
    p.cart_pole.interface.set(DeviceConfig(debug_led=False))


if __name__ == '__main__':
    FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'
    logging.basicConfig(format=FORMAT)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
```
