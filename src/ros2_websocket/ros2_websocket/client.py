import asyncio
import json
from websockets import WebSocketServerProtocol
from rclpy.node import Node
import rclpy.client
from ros2_websocket.protocol_message import ProtocolMessage


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


class Client:

    def __init__(self, id: str, node: Node, conn: WebSocketServerProtocol, caps):
        self.node = node
        self.logger = node.get_logger()
        self.id = id
        self.capabilities = []
        self.operations = {}
        self.event_loop = asyncio.get_event_loop()

        for capability_class in caps:
            self.add_capability(capability_class)

        self._conn = conn

    def run_in_main_loop(self, coroutine_or_callback):
        return self.node.executor.create_task(coroutine_or_callback)

    def run_in_websocket_loop(self, coroutine):
        return self.event_loop.call_soon_threadsafe(
            lambda: self.event_loop.create_task(coroutine))

    def register_operation(self, opcode: int, handler):
        """Register a handler for an opcode
        Keyword arguments:
        opcode  -- the opcode to register this handler for
        handler -- a callback function to call for messages with this opcode
        """
        self.operations[opcode] = handler

    def unregister_operation(self, opcode: int):
        """Unregister a handler for an opcode
        Keyword arguments:
        opcode -- the opcode to unregister the handler for
        """
        if opcode in self.operations:
            del self.operations[opcode]

    def add_capability(self, capability_class):
        """Add a capability to the protocol.
        This method is for convenience; assumes the default capability
        constructor
        Keyword arguments:
        capability_class -- the class of the capability to add
        """
        self.capabilities.append(capability_class(self))

    async def run(self):
        self.log_warn("Connection established.")

        try:
            async for msg in self._conn:
                try:
                    req = ProtocolMessage(msg)
                except Exception as err:
                    self.log_warn(f"Unable to decode message: {str(err)}.")
                    continue

                asyncio.create_task(self._process_request(req))

            self.log(f"Connection closed gracefully.")
        except Exception as err:
            self.log_warn(f"Connection closed unexpectedly: {str(err)}.")
        finally:
            await self.dispose()

    async def send(self, msg):
        await self._conn.send(msg)

    async def _process_request(self, msg: ProtocolMessage):
        # now try to pass message to according operation
        try:
            await self.operations[msg.header.op_code](msg)
        except Exception as exc:
            self.log_error(f"{msg.header.op_code}: {str(exc)}", msg.id)

    async def create_client_async(
            self,
            srv_type,
            srv_name: str) -> rclpy.client.Client:

        client = None

        def run():
            nonlocal client
            client = self.node.create_client(srv_type, srv_name)
        await self.node.executor.create_task(run)
        return client

    async def dispose(self):
        for cap in self.capabilities:
            await cap.dispose()

        self.capabilities.clear()
        self.operations.clear()

    def log_error(self, message, lid=None):
        self.log("error", message, lid)

    def log_info(self, message, lid=None):
        self.log("info", message, lid)

    def log_warn(self, message, lid=None):
        self.log("warn", message, lid)

    def log_debug(self, message, lid=None):
        self.log("debug", message, lid)

    def log(self, level, message, lid=None):
        """Log a message to the client.  By default just sends to stdout
        Keyword arguments:
        level   -- the logger level of this message
        message -- the string message to send to the user
        lid     -- an associated for this log message
        """
        stdout_formatted_msg = None
        if lid is not None:
            stdout_formatted_msg = f"[Client {self.id}] [id: {lid}] {message}"
        else:
            stdout_formatted_msg = f"[Client {self.id}] {message}"

        if level == "error" or level == "err":
            self.logger.error(stdout_formatted_msg)
        elif level == "warning" or level == "warn":
            self.logger.warn(stdout_formatted_msg)
        elif level == "info" or level == "information":
            self.logger.info(stdout_formatted_msg)
        else:
            self.logger.debug(stdout_formatted_msg)
