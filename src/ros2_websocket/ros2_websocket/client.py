import asyncio
import json
import signal
from uuid import uuid4
from websockets import WebSocketServerProtocol
from rclpy.node import Node


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


class Client:

    def __init__(self, node: Node, conn: WebSocketServerProtocol, caps):
        self.node = node
        self.logger = node.get_logger()
        self.id = str(uuid4())
        self.capabilities = []
        self.operations = {}
        self.event_loop = asyncio.get_event_loop()

        for capability_class in caps:
            self.add_capability(capability_class)

        self._conn = conn

    def register_operation(self, opcode: str, handler):
        """Register a handler for an opcode
        Keyword arguments:
        opcode  -- the opcode to register this handler for
        handler -- a callback function to call for messages with this opcode
        """
        self.operations[opcode] = handler

    def unregister_operation(self, opcode: str):
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
                    req = json.loads(msg)
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
        await self._conn.send(json.dumps(msg).encode('utf-8'))

    async def _process_request(self, msg: dict):
        # process fields JSON-message object that "control" rosbridge
        mid = None
        if "id" in msg:
            mid = msg["id"]
        if "op" not in msg:
            if "receiver" in msg:
                self.log_error("Received a rosbridge v1.0 message.")
            else:
                self.log_error(
                    f"Received a message without an op.  All messages require 'op' field with value one of: {list(self.operations.keys())}.",
                    mid,
                )
            return
        op = msg["op"]
        if op not in self.operations:
            self.log_error(
                f"Unknown operation: {op}.  Allowed operations: {list(self.operations.keys())}",
                mid,
            )
            return
        # this way a client can change/overwrite it's active values anytime by just including parameter field in any message sent to rosbridge
        #  maybe need to be improved to bind parameter values to specific operation..
        if "fragment_size" in msg.keys():
            self.fragment_size = msg["fragment_size"]
            # print "fragment size set to:", self.fragment_size
        if "message_interval" in msg.keys() and is_number(msg["message_interval"]):
            self.delay_between_messages = msg["message_interval"]
        if "png" in msg.keys():
            self.png = msg["msg"]

        # now try to pass message to according operation
        try:
            await self.operations[op](msg)
        except Exception as exc:
            self.log_error(f"{op}: {str(exc)}", mid)

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
