import rclpy
from ros2_websocket.websocket_server_node import WebsocketServerNode


def main():
    rclpy.init()
    service = WebsocketServerNode()

    try:
        rclpy.spin(service)
        rclpy.shutdown()
    finally:
        service.shutdown()


if __name__ == '__main__':
    main()
