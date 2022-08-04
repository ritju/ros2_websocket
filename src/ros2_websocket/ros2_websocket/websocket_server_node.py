import asyncio
import signal
from rclpy.node import Node
from threading import Thread
from websockets import WebSocketServerProtocol, serve

from ros2_websocket.caps.call_service import CallService
from ros2_websocket.client import Client
from ros2_websocket.caps.subscribe import Subscribe

class WebsocketServerNode(Node):

    supported_capabilities = [
        CallService,
        # Advertise,
        # Publish,
        Subscribe,
        # Defragment,
        # AdvertiseService,
        # ServiceResponse,
        # UnadvertiseService,
    ]

    def __init__(self):
        super().__init__("websocket_bridge")
        
        self._clients = {}

        self._port = self.declare_parameter("port", 9090).value
        self._host = self.declare_parameter("host", "").value

        self._start_ws_server()

    def _start_ws_server(self):
        Thread(None, lambda:asyncio.run(self._accept())).start()

    async def _create_client(self, websocket : WebSocketServerProtocol, uri):
        cli = Client(self, websocket, self.supported_capabilities)
        self._clients[cli.id] = cli
        
        try:
            await cli.run()
        finally:
            del self._clients[cli.id]

    def shutdown(self):
        self.event_loop.call_soon_threadsafe(self.event_loop.stop)
        
    async def _accept(self):
        self.event_loop = asyncio.get_event_loop()

        self.get_logger().info(
            f'ROS2 Websocket bridge started. Listening on {self._host}:{self._port}.')
        async with serve(self._create_client, self._host, self._port):
            await asyncio.Future()  # run forever 

        