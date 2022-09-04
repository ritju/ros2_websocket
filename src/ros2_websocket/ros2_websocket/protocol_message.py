from ast import And, Call, Sub
import ctypes
from email import header
from inspect import trace
import re
from struct import pack, pack_into, unpack
from tkinter.messagebox import NO
from typing import Any, Type, Union
from uuid import UUID
from roslib_protocol_msgs.msg import *
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
import numpy
PREAMBLE_SIZE = 4


def deserialize(data, type):
    check_for_type_support(type)
    return _rclpy.rclpy_deserialize(data, type)


def serialize(data, type):
    check_for_type_support(type)
    return _rclpy.rclpy_serialize(data, type)


class ProtocolMessage:
    id: UUID
    header: Header
    body: Union[Advertise, CallService, Publish, Subscribe, Unadvertise, Unsubscribe]
    trailer: Union[bytes, None]

    def __init__(self, data: bytes) -> None:
        offset = 0

        (header_size, body_size) = unpack('<HH', data[offset:PREAMBLE_SIZE])
        offset += PREAMBLE_SIZE

        self.header: Header = deserialize(data[offset:], Header)
        offset += header_size

        self.id = UUID(bytes=self.header.id.uuid.tobytes())

        if self.header.op_code == Header.OP_CODE_ADVERTISE:
            self.body = deserialize(data[offset:], Advertise)
        elif self.header.op_code == Header.OP_CODE_CALL_SERVICE:
            self.body = deserialize(
                data[offset:], CallService)
        elif self.header.op_code == Header.OP_CODE_PUBLISH:
            self.body = deserialize(
                data[offset:], Publish)
        elif self.header.op_code == Header.OP_CODE_SUBSCRIBE:
            self.body = deserialize(data[offset:], Subscribe)
        elif self.header.op_code == Header.OP_CODE_UNADVERTISE:
            self.body = deserialize(data[offset:], Unadvertise)
        elif self.header.op_code == Header.OP_CODE_UNSUBSCRIBE:
            self.body = deserialize(data[offset:], Unsubscribe)
        else:
            raise Exception("Unsupported op code {}".format(self.header.op_code))

        offset += body_size

        if isinstance(self.body, (Publish, CallService)):
            self.trailer = data[offset:]
        else:
            self.trailer = None

    def decode_trailer(self, t: Type):
        return deserialize(self.trailer, t)


class OutgoingProtocolMessage:
    header: Header
    body: Union[Publish, ServiceResponse]
    trailer: None

    def __init__(self, id: UUID, body: Union[Publish, ServiceResponse], trailer) -> None:
        if isinstance(body, Publish):
            op_code = Header.OP_CODE_PUBLISH
        elif isinstance(body, ServiceResponse):
            op_code = Header.OP_CODE_SERVICE_RESPONSE
        else:
            raise Exception("Unsupported body type '{}'".format(str(body)))

        self.header = Header()
        self.header.id.uuid = numpy.frombuffer(id.bytes, dtype=numpy.uint8)
        self.header.op_code = op_code

        self.body = body
        self.trailer = trailer

    def encode(self):
        header = serialize(self.header, Header)
        body = serialize(self.body, self.body.__class__)
        if isinstance(self.trailer, bytes):
            trailer = self.trailer
        elif self.trailer != None:
            trailer = serialize(
                self.trailer, self.trailer.__class__)
        else:
            trailer = None

        preamble = pack("<HH", len(header), len(body))

        ret = preamble + header + body
        return ret + trailer if trailer != None else ret
