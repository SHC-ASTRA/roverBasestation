# ROS
import rclpy
import signal
import threading
# Custom ROS class
from ros_handling import RosNode, ros2_thread
# Flask
from flask import Flask, send_from_directory, send_file, request
# Flask SocketIO
from flask_socketio import SocketIO
# Path handling
import os
from pathlib import Path
# Image processing for display
import base64
# OpenCV
import cv2
# NumPy
import numpy as np


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

# Set up SocketIO instance
socketio = SocketIO(app)

# ROS Initialization
rclpy.init(args=None)
ros_node = RosNode()
# ROS2 threading
threading.Thread(target=ros2_thread, args=[ros_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

# Root Page, serves built react page
@app.route('/')
def serve_page():
    data = send_from_directory(app.static_folder, 'index.html')
    try:
        return data
    except:
        return "500: error serving page!"

# API
@app.route('/message_data')
def get_node_data():
    return ros_node.message_data

@app.route('/publish_message')
def get_publish_message():
    ros_node.publish_message()
    return {}

# Socket IO initialization
if __name__ == '__main__':
    socketio.run(app)


# Socket IO message handlers
@socketio.on('message')
def handle_message(data):
    print(f'received message with data \"{data}\" from {request.sid}')

@socketio.on('connect')
def handle_connection():
    print(f"connected user {request.sid}")

@socketio.on('disconnect')
def handle_disconnect():
    print(f'disconnected user {request.sid}')
    ros_node.handle_disconnect(request.sid)

# Handle image subscription request
@socketio.on('image_subscription')
def image_subscription_message(topic_name):
    req_id = request.sid
    handle_image_subscription(req_id, topic_name)

def handle_image_subscription(socket_id, topic_name):
    print(f'Received request to subscribe to {topic_name} for sensor_msgs.msg.CompressedImage')
    def socket_callback(msg):
        # Do some handling for the image to be send over to the
        # to the front-end and displayed in an image in base64
        print("Message received")

        # Convert the message to binary encoding
        current_frame = ros_node.opencv_bridge.compressed_imgmsg_to_cv2(msg)
        frame_data = cv2.imencode('.jpg', current_frame)[1]
        image_bytes = np.array(frame_data).tobytes()

        # Convert bytearray to base64
        base64_str = base64.b64encode(image_bytes).decode()
        # Emit the data to the websockets
        socketio.emit(topic_name,{'data': base64_str})
        
    ros_node.create_image_subscriber(socket_callback, topic_name, socket_id)
    