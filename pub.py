from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from threading import Thread, main_thread
from cartpole.common.interface import State
import asyncio
import json
import time


class FoxgloveWrapper(Thread):
    CARTPOLE_STATE_SCHEMA = {
        "type": "object",
        "properties": {
            "cart_position": {"type": "number"},
            "cart_velocity": {"type": "number"},
            "cart_acceleration": {"type": "number"},
            "pole_angle": {"type": "number"},
            "pole_angular_velocity": {"type": "number"},
        },
    }

    def __init__(self):
        super().__init__(daemon=True)
        self._loop: asyncio.AbstractEventLoop = None
        self._queue: asyncio.Queue = None
        self.start()

    def puplish(self, state: FloatTensor, action: float):
        pair = (state, a)
        asyncio.run_coroutine_threadsafe(self._queue.put(pair), self._loop)
        # print("put from main thread:", payload)

    def run(self):
        self._loop = self._foxglove_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._queue = asyncio.Queue()
        self._loop.run_until_complete(self._async_entrypoint())

    async def _async_entrypoint(self):
        async with FoxgloveServer("0.0.0.0", 8765, "cartpole") as server:
            channel_spec = {
                "topic": "/cartpole/state",
                "encoding": "json",
                "schemaName": "CartPoleState",
                "schema": json.dumps(CARTPOLE_STATE_SCHEMA),
            }
            channel_id = await server.add_channel(channel_spec)
            while main_thread().is_alive():
                state, action = await self._queue.get()
                msg = {
                    'cart_position': state[0],
                    'cart_velocity': state[1],
                    'cart_acceleration': action,
                    'pole_angle': state[2],
                    'pole_angular_velocity': state[3],
                }
                await server.send_message(channel_id, time.time_ns(), json.dumps(payload))


if __name__ == "__main__":
    import math

    fox = FoxgloveWrapper()
    time.sleep(0.1)

    i = 0
    while True:
        state = State(
            cart_position=0.25 * math.sin(i / 10),
            pole_angle=(i / 6) % (2 * math.pi),
        )
        fox.push(state)
        time.sleep(0.1)
        i += 1