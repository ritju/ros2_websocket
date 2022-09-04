from ast import arg
import asyncio
from time import time
import uuid
from rclpy.expand_topic_name import expand_topic_name

from ros2_websocket.client import Client
from ros2_websocket.cap import Cap
from ros2_websocket.internal.message_conversion import populate_instance, extract_values
from ros2_websocket.internal.ros_loader import get_service_class, get_service_request_instance
from ros2_websocket.protocol_message import OutgoingProtocolMessage, ProtocolMessage

import roslib_protocol_msgs.msg as prot


class ServiceNotReadyException(Exception):
    def __init__(self, servicename):
        Exception.__init__(self, "Service %s is not ready." % servicename)


class InvalidServiceException(Exception):
    def __init__(self, servicename):
        Exception.__init__(self, "Service %s does not exist" % servicename)


class ServiceCallTimeoutException(Exception):
    def __init__(self, servicename, timeout):
        Exception.__init__(
            self, f"Service call to '{servicename}' timed out after {timeout} seconds.")


class InvalidServiceResultException(Exception):
    def __init__(self, servicename, result):
        Exception.__init__(
            self, f"Service call to '{servicename}' returned invalid result: {result}.")


class ServiceCallAbortedException(Exception):
    def __init__(self, servicename):
        Exception.__init__(self,
                           "Service call to '%s' was aborted." % servicename)


def trim_servicename(service):
    if "#" in service:
        return service[: service.find("#")]
    return service


def args_to_service_request_instance(service, inst, args):
    """Populate a service request instance with the provided args
    args can be a dictionary of values, or a list, or None
    Propagates any exceptions that may be raised."""
    msg = {}
    if isinstance(args, list):
        msg = dict(zip(inst.get_fields_and_field_types().keys(), args))
    elif isinstance(args, dict):
        msg = args

    # Populate the provided instance, propagating any exceptions
    populate_instance(msg, inst)


def extract_id(service, cid):
    if cid is not None:
        return cid
    elif "#" in service:
        return service[service.find("#") + 1:]


class CallService(Cap):
    def __init__(self, client: Client):
        # Call superclass constructor
        Cap.__init__(self, client)

        self._shutdownSignal = asyncio.Future()

        # Register the operations that this capability provides
        client.register_operation(prot.Header.OP_CODE_CALL_SERVICE, self.call_service)

    async def dispose(self):
        self._shutdownSignal.set_result(None)

    async def call_service(self, message: ProtocolMessage):
        # Pull out the ID
        cid = message.id
        body: prot.CallService = message.body

        # Extract the args
        service = body.service
        type = body.type
        timeout = body.timeout

        try:
            service_class = get_service_class(type)
            args = message.decode_trailer(service_class.Request)
            response = await self._invoke(service, service_class, args, timeout)
        except Exception as err:
            await self._failure(cid, err)
            return

        await self._success(cid, response)

    async def _invoke(self, service: str, service_class, args, timeout: int):
        # Given the service name, fetch the type and class of the service,
        # and a request instance

        client = None
        fut = None
        try:
            notifier = asyncio.Future()

            def start_call():
                nonlocal fut, client
                try:
                    client = self.client.node.create_client(service_class, service)
                    if not client.service_is_ready():
                        raise ServiceNotReadyException(service)
                    fut = client.call_async(args)
                    notifier.set_result(None)
                except Exception as e:
                    notifier.set_exception(e)
            self.client.node.executor.create_task(start_call)
            await notifier

            try:
                w = asyncio.create_task(
                    asyncio.wait_for(fut, timeout if timeout >= 0 else None))
                await asyncio.wait(
                    [self._shutdownSignal, w],
                    return_when=asyncio.FIRST_COMPLETED)

                if w.done():
                    await w
                else:
                    raise ServiceCallAbortedException(service)
            except:
                raise ServiceCallTimeoutException(service, timeout)

            result = fut.result()
            if result is not None:
                return result
            else:
                raise InvalidServiceResultException(service, result)
        finally:
            self.client.run_in_main_loop(
                lambda: self.client.node.destroy_client(client))

    async def _success(self, cid, message):
        body = prot.ServiceResponse()
        body.result = True
        outgoing_message = OutgoingProtocolMessage(cid, body, message)

        await self.client.send(outgoing_message.encode())

    async def _failure(self, cid, exc):
        self.client.log_error("call_service %s: %s" % (type(exc).__name__, str(exc)), cid)
        # send response with result: false
        body = prot.ServiceResponse()
        body.result = False
        outgoing_message = OutgoingProtocolMessage(cid, body, None)
        await self.client.send(outgoing_message.encode())
