import asyncio
import json
import uuid

from rclpy import Node
from rclpy.clock import ROSClock
from websockets import WebSocketServerProtocol


class ClientManager:
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()
        self._clients = {}

    async def _receive(self, id:str, client: WebSocketServerProtocol):
        async for msg in client:
            try:
                req = json.loads(msg)
            except Exception as err:
                self._logger.warn("[{}] Unable to decode message: {}.".format(id, err))
                continue
            req
            
    def add_client(self, client: WebSocketServerProtocol):
        client_id = uuid.uuid4().__str__()
        self._clients[client_id] = client
        self._logger.info("Accepted client connection [{}]".format(client_id))
        
        asyncio.create_task(self._receive(client))
