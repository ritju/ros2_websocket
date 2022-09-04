# Subscribe a topic.

string topic         # Topic to subscribe.
string type          # Type of the topic.
int32 throttle_rate  # How fast should the server send messages (per second), 0 means no limit.
QosOption qos        # QoS options.