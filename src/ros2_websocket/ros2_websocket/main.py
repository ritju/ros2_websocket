import rclpy
from ros2_websocket.websocket_server_node import WebsocketServerNode
import ros2_websocket.internal.ros_loader as ros_loader
import nav2_msgs.action
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