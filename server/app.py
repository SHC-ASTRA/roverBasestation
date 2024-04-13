# ROS
import rclpy
import signal
import threading
# Custom ROS class
from ros_handling import RosNode, ros2_thread
# Flask
from flask import Flask, send_from_directory
# Path handling
import os
from pathlib import Path


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


# Static folder is not equivalent to the static react build folder. React stores static build in 
app = Flask(__name__, static_folder=Path(f"{os.getcwd()}/../react-app/build/"), static_url_path='/')

# ROS Initialization
rclpy.init(args=None)
node = RosNode()
# ROS2 threading
threading.Thread(target=ros2_thread, args=[node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

@app.route('/')
def serve_page():
    data = send_from_directory(app.static_folder, 'index.html')
    try:
        return data
    except:
        return "500: error serving page!"

@app.route('/message_data')
def get_node_data():
    return node.message_data

@app.route('/publish_message')
def get_publish_message():
    node.publish_message()
    return {}