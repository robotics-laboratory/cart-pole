import logging
from pathlib import Path

from aiohttp import web

from sessions.collector import SessionData
from common.util import find_free_port
import IPython.display as ipd
from threading import Thread
import dataclasses
import asyncio


class SimpleServer:
    FRONTEND_PATH = Path('web_view/frontend')

    def __init__(self, session_data: SessionData = None, port: int = 8080):
        self.port = port
        self.data: SessionData = session_data
        self.thread = Thread(target=self.run)

    async def index(self, _):
        return web.FileResponse(self.FRONTEND_PATH / 'index.html')

    async def ws_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        data = dataclasses.asdict(self.data)
        await ws.send_json(data)
        return ws

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = web.Application()
        app.add_routes([
            web.get('/', self.index),
            web.get('/ws', self.ws_handler),
            web.static('/static', str(self.FRONTEND_PATH)),
        ])

        web.run_app(app, host='0.0.0.0', port=self.port, access_log=None, print=False)


def session_widget(session_data: SessionData, height=None):
    server = getattr(session_widget, '__server_instance', None)
    if server is None:
        server = SimpleServer(session_data, port=find_free_port())
        thread = Thread(target=server.run)
        thread.start()
        setattr(session_widget, '__server_instance', server)
    server.data = session_data
    src = f'http://localhost:{server.port}/?id=_'
    print('IFrame URL:', src)
    if height is None:
        height = len(session_data.groups) * 300 + 100
    return ipd.IFrame(src=src, width='', height=height, extras=['style="width: 100%"'])
