# Main ROS2 library / bindings
import rclpy
# ROS2 node class
from rclpy.node import Node
# ROS2 standard (topic) messages
import std_msgs.msg

class RosNode(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(std_msgs.msg.String, 'flask_pub_topic', 10)
        self.subscription = self.create_subscription(
            std_msgs.msg.String,
            '/topic',
            self.chatter_callback,
            10)
        self.latest_message = None

    def chatter_callback(self, msg):
        print(f'chatter cb received: {msg.data}')
        self.latest_message = msg.data

    def publish_message(self):
        msg = String()
        msg.data = 'hello, world!'
        self.publisher.publish(msg)


def ros2_thread(node):
    print('Entering Ros2 thread')
    rclpy.spin(node)
    print('Leaving Ros2 thread')