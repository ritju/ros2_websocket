import asyncio
from rclpy.node import Node
from threading import Thread
from websockets import WebSocketServerProtocol, serve

from ros2_websocket.caps.call_service import CallService
from ros2_websocket.client import Client
from ros2_websocket.caps.subscribe import Subscribe
from ros2_websocket.caps.publisher import Publisher


class WebsocketServerNode(Node):

    supported_capabilities = [
        CallService,
        Publisher,
        Subscribe
    ]

    def __init__(self):
        super().__init__("websocket_bridge")

        self._clients = {}
        self._id = 1
        self._port = self.declare_parameter("port", 9090).value
        self._host = self.declare_parameter("host", "").value

        if not "_publishers" in self.__dict__:
            self._publishers = self._Node__publishers
            self._subscriptions = self._Node__subscriptions
            self._clients = self._Node__clients

        self.create_timer(30, self._timer_callback)
        self._start_ws_server()

    def _timer_callback(self):
        self.get_logger().info(
            f"puslishers: {len(self._publishers)}, "
            f"subscriptions: {len(self._subscriptions)}, "
            f"service_clients: {len(self._clients)}")

    def _start_ws_server(self):
        Thread(None, lambda: asyncio.run(self._accept())).start()

    async def _create_client(self, websocket: WebSocketServerProtocol, uri):
        cli = Client(str(self._id), self, websocket, self.supported_capabilities)
        self._clients[cli.id] = cli

        self._id = self._id + 1

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
            await asyncio.Future()
