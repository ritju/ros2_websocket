import asyncio
from time import time
from rclpy.expand_topic_name import expand_topic_name

from ros2_websocket.client import Client
from ros2_websocket.cap import Cap
from ros2_websocket.internal.message_conversion import populate_instance, extract_values
from ros2_websocket.internal.ros_loader import get_service_class, get_service_request_instance

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
    call_service_msg_fields = [
        (True, "service", str),
        (False, "type", (str,type(None))),
        (False, "fragment_size", (int, type(None))),
        (False, "compression", str),
        (False, "timeout", (float,int)),
    ]

    def __init__(self, client: Client):
        # Call superclass constructor
        Cap.__init__(self, client)

        # Register the operations that this capability provides
        client.register_operation("call_service", self.call_service)

    async def call_service(self, message: dict):
        # Pull out the ID
        cid = message.get("id", None)

        # Typecheck the args
        self.basic_type_check(message, self.call_service_msg_fields)

        # Extract the args
        service = message["service"]
        type = message.get("type", None)
        timeout = message.get("timeout", 60.0)
        fragment_size = message.get("fragment_size", None)
        compression = message.get("compression", "none")
        args = message.get("args", [])

        # Check for deprecated service ID, eg. /rosbridge/topics#33
        cid = extract_id(service, cid)

        try:
            response = await self._invoke(service,type, args, timeout)
        except Exception as err:
            await self._failure(cid, service, err)
            return

        await self._success(cid, service, response)

    async def _invoke(self, service: str, service_type:str, args: list, timeout: int):
        # Given the service name, fetch the type and class of the service,
        # and a request instance

        if service_type is None:
            # This should be equivalent to rospy.resolve_name.
            service = expand_topic_name(service, self.client.node.get_name(),
                                        self.client.node.get_namespace())

            service_names_and_types = dict(self.client.node.get_service_names_and_types())
            service_type = service_names_and_types.get(service)
            if service_type is None:
                raise InvalidServiceException(service)
            # service_type is a tuple of types at this point; only one type is supported.
            if len(service_type) > 1:
                self.client.log_warn(f"More than one service type detected: {service_type}")
            service_type = service_type[0]

        service_class = get_service_class(service_type)
        inst = get_service_request_instance(service_type)

        # Populate the instance with the provided args
        args_to_service_request_instance(service, inst, args)

        client = self.client.node.create_client(service_class, service)
        if not client.service_is_ready():
            raise ServiceNotReadyException(service)

        fut = client.call_async(inst)

        try:
            if timeout >= 0:
                try:
                    await asyncio.wait_for(fut, timeout)
                except Exception:
                    client.remove_pending_request(fut)
                    raise ServiceCallTimeoutException(service, timeout)
            else:
                await fut

            result = fut.result()
            if result is not None:
                # Turn the response into JSON and pass to the callback
                json_response = extract_values(result)
            else:
                raise InvalidServiceResultException(service, result)

            return json_response
        finally:
            self.client.run_in_main_loop(
                lambda: self.client.node.destroy_client(client))
            

    async def _success(self, cid, service, message):
        outgoing_message = {
            "op": "service_response",
            "service": service,
            "values": message,
            "result": True,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        await self.client.send(outgoing_message)

    async def _failure(self, cid, service, exc):
        self.client.log_error("call_service %s: %s" % (type(exc).__name__, str(exc)), cid)
        # send response with result: false
        outgoing_message = {
            "op": "service_response",
            "service": service,
            "values": str(exc),
            "result": False,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        await self.client.send(outgoing_message)
