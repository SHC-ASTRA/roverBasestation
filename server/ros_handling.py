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

FEEDBACK_TOPIC = "/astra/core/feedback"
CHATTER_TOPIC = "/topic"

class RosNode(Node):
    # Dictionary of Publishers
    publishers = {}
    # Dictionary of Subscribers
    subscribers = {}
    # Dictionary of Image Subscribers and other data
    # Purge this as disconnects are done processed
    # should also store who is listening to these subscribers
    image_subscribers = {}
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
        self.create_subscriber(CHATTER_TOPIC, self.chatter_callback)

        # LiveData subscriber
        self.create_subscriber(FEEDBACK_TOPIC, self.core_feedback_callback)

        self.message_data = {}

        # OpenCV Bridge
        self.opencv_bridge = CvBridge()

    # Subscriber Callbacks

    def chatter_callback(self, msg):
        print(f'chatter cb received: {msg.data}')
        self.message_data[CHATTER_TOPIC] = msg.data

    def core_feedback_callback(self, msg):
        print(f"Recieved data from feedback topic: {msg.data}")
        # Check if the key is defined in the dictionary
        try:
            self.message_data[FEEDBACK_TOPIC]
        except:
            self.message_data[FEEDBACK_TOPIC] = []
        # Append to the key's list
        self.message_data[FEEDBACK_TOPIC].append(msg.data)

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

        for key in self.image_subscribers.keys():
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
        self.destroy_subscription(self.image_subscribers[image_topic]['subscriber'])

# Thread worker that spins the node
def ros2_thread(node):
    print('Entering Ros2 thread')
    rclpy.spin(node)
    print('Leaving Ros2 thread')