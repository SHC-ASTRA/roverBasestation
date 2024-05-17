# Main ROS2 library / bindings
import rclpy
# ROS2 node class
from rclpy.node import Node
# ROS2 standard (topic) messages
import std_msgs.msg
# ROS2 sensor (topic) messages
import sensor_msgs.msg
# ROS2 standard (service) srvs
import std_srvs.srv
# OpenCV
from cv_bridge import CvBridge

TOPIC_LIST = {
    'FEEDBACK_TOPIC': "/astra/core/feedback",
    'CONTROL_TOPIC': "/astra/core/control",
    'CHATTER_TOPIC': "/topic"
}

class RosNode(Node):
    # Dictionary of Publishers
    publishers = {}
    # Dictionary of Subscribers
    subscribers = {}
    
    # Dictionary of Image Subscribers and other data

    # This is cleaned as disconnects are processed
    # Stores who is listening to these subscribers
    image_subscribers = {}
    # ROS Topic names are used as the key for another dictionary
    # that stores the callback, sockets who have requested the subscriber,
    # and the ROS instance of the subscriber 
    """
    {
        topic: {
            callback: subscriber_callback
            socket_ids: [ids of sockets using this callback]
            subscriber: subscriber_instance
        }
    }"""

    def __init__(self):
        super().__init__('astra_base')

        # Basic Subscribers for testing
        self.create_string_publisher("flask_pub_topic")
        self.create_subscriber(TOPIC_LIST['CHATTER_TOPIC'], self.chatter_callback)

        # LiveData subscriber
        self.create_subscriber(TOPIC_LIST['FEEDBACK_TOPIC'], self.core_feedback_callback)

        self.message_data = {}

        # OpenCV Bridge
        self.opencv_bridge = CvBridge()

    # Subscriber Callbacks

    def chatter_callback(self, msg):
        print(f'chatter cb received: {msg.data}')
        self.message_data[TOPIC_LIST['CHATTER_TOPIC']] = msg.data

    def core_feedback_callback(self, msg):
        print(f"Received data from feedback topic: {msg.data}")
        self.append_key_list(TOPIC_LIST['FEEDBACK_TOPIC'], msg)

    def core_control_callback(self, msg):
        print(f"Received data from control topic: {msg.data}")
        self.append_key_list(TOPIC_LIST['CONTROL_TOPIC'], msg)

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
    def publish_string_data(self, topic_name, str_data):
        ros_msg = std_msgs.msg.String()
        ros_msg.data = str_data
        self.publishers[topic_name].publish(ros_msg)
        print(f"Publishing data to \"{topic_name}\": {str_data}")

    # Create a subscriber with a given callback
    def create_subscriber(self, topic_name: str, callback):
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
        self.core_health_service = self.create_service(
            std_srvs.srv.Trigger,
            '/astra/core/health',
            service_callback
        )
        self.arm_health_service = self.create_service(
            std_srvs.srv.Trigger,
            '/astra/arm/health',
            service_callback
        )

    # Subscribe to a COMPRESSED IMAGE topic
    def create_image_subscriber(self, subscriber_callback, image_topic, socketid):
        if image_topic in self.image_subscribers.keys():
            # If the socket exists, update this list of socket ids that are connected
            self.image_subscribers[image_topic]["socket_ids"].append(socketid)
        else:
            self.image_subscribers[image_topic] = {
                'callback': subscriber_callback,
                'socket_ids': [socketid],
                'subscriber': self.create_subscription(
                    sensor_msgs.msg.CompressedImage,
                    image_topic,
                    subscriber_callback,
                    10
                )
            }
        # Introduce some logic that handles recreating the subscriber with BOTH callbacks
        # if a subscriber already exists

    def handle_disconnect(self, socketid):
        # Handle disconnections by removing the connection from subscriptions
        # with more than one user, and delete subscriptions
        # where the connection was the only user
        for key in list(self.image_subscribers.keys()):
            # If this socket uses this subscriber
            if socketid in self.image_subscribers[key]["socket_ids"]:
                # Kill the subscription if this is the last socket
                # that is making use of this subscriber
                if len(self.image_subscribers[key]["socket_ids"]) == 1:
                    self.kill_image_subscriber(key)
                # If there are multiple sockets using this subscriber,
                # remove the socket's id from the list
                else:
                    self.image_subscribers[key]["socket_ids"].remove(socketid)

    # If all of the request makers are gone, handle deleting
    def kill_image_subscriber(self, image_topic):
        # Destroy the ROS subscription to conserve resources
        self.destroy_subscription(self.image_subscribers[image_topic]['subscriber'])
        # Remove the key entry for the subscription
        # so that it may be recreated in the future
        del self.image_subscribers[image_topic]

# Thread worker that spins the node
def ros2_thread(node):
    print('Entering Ros2 thread')
    rclpy.spin(node)
    print('Leaving Ros2 thread')