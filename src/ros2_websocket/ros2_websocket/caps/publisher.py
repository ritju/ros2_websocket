import asyncio
from asyncio import Future, futures
from ros2_websocket.client import Client
from ros2_websocket.cap import Cap
from ros2_websocket.exceptions import TopicNotEstablishedException, TypeConflictException
from ros2_websocket.internal import ros_loader
from ros2_websocket.internal.message_conversion import msg_class_type_repr, populate_instance
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class PublisherContext:
    def __init__(self, client: Client, id: str, topic: str, msg_type: str,
                 depth: int, reliability: int, durability: int) -> None:
        self.id = id
        self.topic = topic
        self._client = client

        # First check to see if the topic is already established
        topics_names_and_types = dict(client.node.get_topic_names_and_types())
        topic_type = topics_names_and_types.get(topic)

        # If it's not established and no type was specified, exception
        if msg_type is None and topic_type is None:
            raise TopicNotEstablishedException(topic)

        # topic_type is a list of types or None at this point; only one type is supported.
        if topic_type is not None:
            if len(topic_type) > 1:
                self.client.log_warn(f"More than one topic type detected: {topic_type}")
            topic_type = topic_type[0]

        # Use the established topic type if none was specified
        if msg_type is None:
            msg_type = topic_type

        # Load the message class, propagating any exceptions from bad msg types
        msg_class = ros_loader.get_message_class(msg_type)

        # Make sure the specified msg type and established msg type are same
        msg_type_string = msg_class_type_repr(msg_class)
        if topic_type is not None and topic_type != msg_type_string:
            raise TypeConflictException(topic, topic_type, msg_type_string)

        # Adding a lifespan solves the problem of late-joining subscribers
        # without the need of a custom message publisher implementation.
        publisher_qos = QoSProfile(
            depth=depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        if durability is not None:
            publisher_qos.durability = durability

        if reliability is not None:
            publisher_qos.reliability = reliability

        self._msg_class = msg_class
        self._handle = client.node.create_publisher(msg_class, topic, qos_profile=publisher_qos)

    def publish(self, msg):
        # Create a message instance
        inst = self._msg_class()

        # Populate the instance, propagating any exceptions that may be thrown
        populate_instance(msg, inst)

        # Publish the message
        self._handle.publish(inst)

    def dispose(self):
        self._client.run_in_main_loop(
            lambda: self._client.node.destroy_publisher(self._handle))


class Publisher(Cap):
    advertise_msg_fields = [
        (True, "topic", str),
        (True, "id", str),
        (True, "type", str),
        (False, "depth", int),
        (False, "qos_reliability", (int, type(None))),
        (False, "qos_durability", (int, type(None)))
    ]

    unadvertise_msg_fields = [(True, "id", str)]
    publish_msg_fields = [(True, "id", str)]

    def __init__(self, client):
        # Call superclass constructor
        Cap.__init__(self, client)

        # Register the operations that this capability provides
        client.register_operation("advertise", self.advertise)
        client.register_operation("unadvertise", self.unadvertise)
        client.register_operation("publish", self.publish)

        # Initialize class variables
        self._publishers = {}

    async def advertise(self, msg: dict):
        self.basic_type_check(msg, self.advertise_msg_fields)
        id = msg["id"]
        topic = msg["topic"]
        msg_type = msg["type"]
        reliability = msg.get("qos_reliability", None)
        durability = msg.get("qos_durability", None)
        queue_size = msg.get("depth", 0)

        if id not in self._publishers:
            self._publishers[id] = PublisherContext(self.client,
                                                    id, topic, msg_type, queue_size, reliability, durability)
            self.client.log_info(f"Created publisher for '{topic}'.", id)
        else:
            self.client.log_warn(
                f"Unable to advertise topic '{topic}', duplicate publisher id.", id)

    async def unadvertise(self, msg: dict):
        id = msg["id"]
        if msg["id"] in self._publishers:
            self._dispose_publihser(self._publishers[id])
            del self._publishers[id]
        else:
            self.client.log_warn(f"Unable to unadvertise topic, unknown publisher id.", id)

    async def publish(self, msg: dict):
        # Do basic type checking
        self.basic_type_check(msg, self.publish_msg_fields)

        id = msg["id"]
        if id in self._publishers:
            m = msg.get("msg", {})
            self._publishers[id].publish(m)
        else:
            self.client.log_warn(f"Unable to publish message, unknown publisher id.", id)

    async def dispose(self):
        for pub in self._publishers.values():
            self._dispose_publihser(pub)
        self._publishers.clear()

    def _dispose_publihser(self, pub: PublisherContext):
        pub.dispose()
        self.client.log_info(f"Destroyed publisher for '{pub.topic}'.", pub.id)
