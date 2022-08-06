
from functools import partial
from time import time
from rclpy.qos import QoSProfile
from ros2_websocket.cap import Cap
from ros2_websocket.client import Client
from ros2_websocket.internal.message_conversion import extract_values, msg_class_type_repr
import ros2_websocket.internal.ros_loader as ros_loader


class SubscriptionInfo:
    def __init__(self, client: Client, cb,
                 sid: str, topic: str, msg_type, throttle_rate: int,
                 queue_size: int, durability: int, reliability: int) -> None:
        topics_names_and_types = dict(client.node.get_topic_names_and_types())
        topic_type = topics_names_and_types.get(topic)

        if msg_type is None and topic_type is None:
            raise TopicNotEstablishedException(topic)

          # topic_type is a list of types or None at this point; only one type is supported.
        if topic_type is not None:
            if len(topic_type) > 1:
                client.log_warn(f"More than one topic type detected: {topic_type}", sid)
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

        # Certain combinations of publisher and subscriber QoS parameters are
        # incompatible. Here we make a "best effort" attempt to match existing
        # publishers for the requested topic. This is not perfect because more
        # publishers may come online after our subscriber is set up, but we try
        # to provide sane defaults. For more information, see:
        # - https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
        # - https://github.com/RobotWebTools/rosbridge_suite/issues/551
        # qos = QoSProfile(
        #     depth=queue_size,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     reliability=ReliabilityPolicy.RELIABLE,
        # )

        # infos = client.node.get_publishers_info_by_topic(topic)
        # if any(pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for pub in infos):
        #     qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # if any(pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for pub in infos):
        #     qos.reliability = ReliabilityPolicy.BEST_EFFORT

        qos = QoSProfile(
            depth=queue_size
        )

        if durability is not None:
            qos.durability = durability
        if reliability is not None:
            qos.reliability = reliability

        self.id = sid
        self.topic = topic
        self.throttle_rate = throttle_rate / 1000.0
        self.last_send_time = time()

        self._client = client

        def subscribe():
            self._handle = client.node.create_subscription(
                msg_class, topic, partial(cb, self), qos_profile=qos)
        client.run_in_main_loop(subscribe)

    def dispose(self):
        self._client.run_in_main_loop(
            lambda: self._client.node.destroy_subscription(self._handle))


class Subscribe(Cap):
    subscribe_msg_fields = [
        (True, "id", str),
        (True, "topic", str),
        (False, "type", str),
        (False, "throttle_rate", int),
        (False, "fragment_size", int),
        (False, "depth", int),
        (False, "qos_reliability", (int, type(None))),
        (False, "qos_durability", (int, type(None))),
        (False, "compression", str),
    ]
    unsubscribe_msg_fields = [(True, "id", str)]

    def __init__(self, client: Client):
        # Call superclass constructor
        Cap.__init__(self, client)

        # Register the operations that this capability provides
        client.register_operation("subscribe", self.subscribe)
        client.register_operation("unsubscribe", self.unsubscribe)

        self._subscriptions = {}

    def _msg_callback(self, info: SubscriptionInfo, msg):
        now = time()
        if info.throttle_rate > 0 and now - info.last_send_time < info.throttle_rate:
            return
        info.last_send_time = now

        json = {
            "op": "publish",
            "id": info.id,
            "topic": info.topic,
            "msg": extract_values(msg)
        }

        self.client.run_in_websocket_loop(self._send_to_client(info.id, json))

    async def _send_to_client(self, sid, json):
        try:
            await self.client.send(json)
        except Exception as err:
            self.client.log_warn(f'Unable to send topic message to client: {str(err)}', sid)

    async def subscribe(self, msg):
        # Check the args
        self.basic_type_check(msg, self.subscribe_msg_fields)

        # Pull out the ID
        sid = msg["id"]

        # Make the subscription
        topic = msg["topic"]

        msg_type = msg.get("type", None)
        throttle_rate = msg.get("throttle_rate", 0)
        fragment_size = msg.get("fragment_size", None)
        queue_length = msg.get("depth", 0)
        reliability = msg.get("qos_reliability", None)
        durability = msg.get("qos_durability", None)

        self._subscriptions[sid] = SubscriptionInfo(
            self.client, self._msg_callback, sid, topic,
            msg_type, throttle_rate, queue_length, durability, reliability)

        self.client.log_info(f"Subscribed to '{topic}'.", sid)

    async def unsubscribe(self, msg):
        sid = msg.get("id", None)
        if sid in self._subscriptions:
            self._dispose_subscription(self._subscriptions[sid])
            del self._subscriptions[sid]

    def _dispose_subscription(self, sub: SubscriptionInfo):
        sub.dispose()
        self.client.log_info(f"Unsubscribed from '{sub.topic}'.", sub.id)

    async def dispose(self):
        for sub in self._subscriptions.values():
            self._dispose_subscription(sub)
        self._subscriptions.clear()
