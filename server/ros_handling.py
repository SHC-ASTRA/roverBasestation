# Main ROS2 library / bindings
import rclpy
# ROS2 node class
from rclpy.node import Node
# ROS2 standard (topic) messages
import std_msgs.msg
# ROS2 standard (service) srvs
import std_srvs.srv

FEEDBACK_TOPIC = "/astra/core/feedback"
CONTROL_TOPIC = "/astra/core/control"
CHATTER_TOPIC = "/topic"

class RosNode(Node):
    # Dictionary of Publishers
    publishers = {}
    # Dictionary of Subscribers
    subscribers = {}
    
    def __init__(self):
        super().__init__('astra_base')

        # Basic Subscribers for testing
        self.create_string_publisher("flask_pub_topic")
        self.create_subscriber(CHATTER_TOPIC, self.chatter_callback)

        # LiveData subscriber
        self.create_subscriber(FEEDBACK_TOPIC, self.core_feedback_callback)

        self.message_data = {}

    # Subscriber Callbacks

    def chatter_callback(self, msg):
        print(f'chatter cb received: {msg.data}')
        self.message_data[CHATTER_TOPIC] = msg.data

    def core_feedback_callback(self, msg):
        print(f"Received data from feedback topic: {msg.data}")
        self.append_key_list(FEEDBACK_TOPIC, msg)

    def core_control_callback(self, msg):
        print(f"Received data from control topic: {msg.data}")
        self.append_key_list(CONTROL_TOPIC, msg)

    ## Helper Functions

    # Create a ROS publisher of String type
    def create_string_publisher(self, topic_name):
        self.publishers[topic_name] = self.create_publisher(
            std_msgs.msg.String,
            topic_name,
            0
        )
        return self.publishers[topic_name]

    # Publish data to a String topic
    def publish_string_data(self, topic_name, message):
        self.publishers[topic_name].publish(message)
        print(f"Publishing data: {message}")

    # Create a subscriber with a given callback
    def create_subscriber(self, topic_name, callback):
        self.create_subscription(
            std_msgs.msg.String,
            topic_name,
            callback,
            0
        )

    def append_key_list(self, topic, msg):
        # Check if key is in dictionary
        try:
            self.message_data[topic]
        except KeyError:
            self.message_data[topic] = []
        # Append to the key's list
        self.message_data[topic].append(msg.data)

    # Health Packet Service

    def initalize_health_packets(self, service_callback):
        this.core_health_service = this.create_service(
            std_srvs.srv.Trigger,
            '/astra/core/health',
            service_callback
        )
        this.arm_health_service = this.create_service(
            std_srvs.srv.Trigger,
            '/astra/arm/health',
            service_callback
        )

# Thread worker that spins the node
def ros2_thread(node):
    print('Entering Ros2 thread')
    rclpy.spin(node)
    print('Leaving Ros2 thread')