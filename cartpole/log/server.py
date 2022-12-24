import asyncio
import json
import logging
import time

from threading import Event, Thread
from cartpole import State
from foxglove_websocket.server import FoxgloveServer
from mcap.writer import Writer


state_schema = {
    "type": "object",
    "properties": {
        "cart_position": {"type": "number"},
        "cart_velocity": {"type": "number"},
        "cart_acceleration": {"type": "number"},
        "pole_angle": {"type": "number"},
        "pole_angular_velocity": {"type": "number"},
    },
}

state_schema_name = 'CartPoleState'
state_topic = "/cartpole/state"

actor_critic_schema = {
    "type": "object",
    "properties": {
        "actor_critic_reward": {"type": "number"},
        "critic_loss": {"type": "number"},
        "reward": {"type": "number"},
    },
}

actor_critic_schema_name = 'ActorCriticState'
actor_critic_topic = "/cartpole/actor_critic"


def get_logger():
    logger = logging.getLogger("LogServer")
    logger.setLevel(logging.DEBUG)

    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s: [%(levelname)s] %(message)s"))
    logger.addHandler(handler)

    return logger


async def _async_entrypoint(queue, stop):  
    async with FoxgloveServer("0.0.0.0", 8765, "CartPole", logger=get_logger()) as server:
        topic_to_id = {}

        async def add(schema_name, schema, topic):
            spec = {
                "topic": topic,
                "encoding": "json",
                "schemaName": schema_name,
                "schema": json.dumps(schema),
            }

            channel_id = await server.add_channel(spec)
            topic_to_id[topic] = channel_id

        await add(state_schema_name, state_schema, state_topic)
        await add(actor_critic_schema_name, actor_critic_schema, actor_critic_topic)

        while not stop.is_set():
            topic, payload = await queue.get()
            await server.send_message(topic_to_id[topic], time.time_ns(), payload)

def foxglove_main(loop, queue, stop):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(_async_entrypoint(queue, stop))

class LogServer:
    def __init__(self, log_path=None):
        self._loop = asyncio.new_event_loop()
        self._queue = asyncio.Queue()
        self._stop = Event()
        self._writer = None

        self._state_schema_id = None
        self._state_channel_id = None

        self.actor_critic_channel_id = None
        self.actor_critic_channel_id = None

        if log_path:
            self.init_writer(log_path)

        self._foxlgove_thread = Thread(
            target=foxglove_main,
            name='foxglove_main_loop',
            daemon=True,
            args=(self._loop, self._queue, self._stop))

        self._foxlgove_thread.start()

    def init_writer(self, log_path):
        self._writer = Writer(open(log_path, "wb"))

        def register(schema_name, schema, topic):
            schema_id = self._writer.register_schema(
                name=schema_name,
                encoding="jsonschema",
                data=json.dumps(schema).encode())
    
            channel_id = self._writer.register_channel(
                schema_id=schema_id,
                topic=topic,
                message_encoding="json")

            return schema_id, channel_id

        self._state_schema_id, self._state_channel_id = register(
            state_schema_name, state_schema, state_topic)

        self._actor_ctiric_schema_id, self._actor_critic_channel_id = register(
            actor_critic_schema_name, actor_critic_schema, actor_critic_topic)

        self._writer.start()


    def publish_state(self, state: State):
        msg = {
            'cart_position': state.cart_position,
            'cart_velocity': state.cart_velocity,
            'cart_acceleration': state.cart_acceleration,
            'pole_angle': state.pole_angle,
            'pole_angular_velocity': state.pole_angular_velocity
        }

        payload = json.dumps(msg).encode()

        if self._writer:
            time_ns = time.time_ns()

            self._writer.add_message(
                channel_id=self._state_channel_id,
                log_time=time_ns,
                data=payload,
                publish_time=time_ns)

        payload_with_topic = (state_topic, payload)
        asyncio.run_coroutine_threadsafe(self._queue.put(payload_with_topic), self._loop)

    def publish_actor_critic(self, actor_critic_reward, critic_loss, reward):
        msg = {
            'actor_critic_reward': actor_critic_reward,
            'critic_loss': critic_loss,
            'reward': reward,
        }

        payload = json.dumps(msg).encode()
        payload_with_topic = (actor_critic_topic, payload)

        if self._writer:
            time_ns = time.time_ns()

            self._writer.add_message(
                channel_id=self._actor_critic_channel_id,
                log_time=time_ns,
                data=payload,
                publish_time=time_ns)

        asyncio.run_coroutine_threadsafe(self._queue.put(payload_with_topic), self._loop)

    def __del__(self):
        if self._writer:
            self._writer.finish()

        self._stop.set()
