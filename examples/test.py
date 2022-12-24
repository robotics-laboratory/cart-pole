import asyncio
import json
import time
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer
from foxglove_websocket.types import ChannelId
from threading import Thread
from queue import Queue


async def foxglove_server_loop(state_queue: Queue):
    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        channel_id = await server.add_channel(
            {
                "topic": "/cartpole/state",
                "encoding": "json",
                "schemaName": "CartPoleState",
                "schema": json.dumps(
                    {
                        "type": "object",
                        "properties": {
                            "cart_position": {"type": "number"},
                            "cart_velocity": {"type": "number"},
                            "cart_acceleration": {"type": "number"},
                            "pole_angle": {"type": "number"},
                            "pole_angular_velocity": {"type": "number"},
                        },
                    }
                ),
            }
        )

        while True:
            await asyncio.sleep(1)



            await server.send_message(channel_id, time.time_ns(), json.dumps())


class CartPoleServer:
    def __init__(self):
        self._state_queue = Queue(10)
        self._foxlgove_thread = Thread(target=foxglove_server_loop, args=(self._state_queue,))
        self._foxlgove_thread.start()

    def publish(self, state, action) -> None:
        # wait while server process queue
        self._msg_queue.put((state, action), block=True)



def foxglove_main_thread(queue: Queue):
    run_cancellable(foxglove_server_loop)

class FoxgloveServerHolder:
    def __init__(self):
        self.foxglove_thread = Thread(target=foxglove_main_thread)
        self.foxglove_thread.start()

if __name__ == "__main__":
    for _ in range(30):
        time.sleep(1)
        print("hello")