import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
import threading
from flask import Flask, send_from_directory, send_file
# Path handling
import os
from pathlib import Path


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, 'flask_pub_topic', 10)
        self.subscription = self.create_subscription(
            String,
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
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal, frame)


rclpy.init(args=None)
ros2_node = TestPublisher()
# Static folder is not equivalent to the static react build folder. React stores static build in 
app = Flask(__name__, static_folder=Path(f"{os.getcwd()}/../react-app/build/"), static_url_path='/')
threading.Thread(target=ros2_thread, args=[ros2_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

# TODO
@app.route('/')
def serve_page():
    data = send_from_directory(app.static_folder, 'index.html')
    try:
        return data
    except:
        return "Error serving page!"

@app.route('/latest_message')
def get_current_time():
    return {'message': ros2_node.latest_message}

@app.route('/publish_message')
def get_publish_message():
    ros2_node.publish_message()
    return {}