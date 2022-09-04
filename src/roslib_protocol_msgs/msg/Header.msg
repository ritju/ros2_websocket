# Header of protocol message.
# Body of the message is specified by 'op_code'.

uint8 OP_CODE_ADVERTISE = 1         # The header follows an Advertise message.
uint8 OP_CODE_UNADVERTISE = 2       # The header follows an Unadvertise message.
uint8 OP_CODE_SUBSCRIBE = 3         # The header follows a Subscribe message.
uint8 OP_CODE_UNSUBSCRIBE = 4       # The header follows an Unsubscribe message.
uint8 OP_CODE_PUBLISH = 5           # The header follows a Publish message.
uint8 OP_CODE_CALL_SERVICE = 6      # The header follows an CallService message.
uint8 OP_CODE_SERVICE_RESPONSE = 7  # The header follows an Service message.

unique_identifier_msgs/UUID id      # ID of this message.
uint8 op_code                       # Type of following message body.