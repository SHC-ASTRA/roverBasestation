# ROS
import rclpy
import signal
import threading
# Custom ROS class
from ros_handling import RosNode, ros2_thread
from ros_handling import CORE_CONTROL, CORE_FEEDBACK, AUTO_CONTROL, ARM_CONTROL, ARM_FEEDBACK, ARM_COMMAND, BIO_CONTROL, BIO_FEEDBACK, FAERIE_CONTROL, FAERIE_FEEDBACK
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
import sys
import time

# Insert the installation direction into the local path
# so that message files can be imported
# Equivalent to sourcing the directory prior
sys.path.insert(1, 'ros_msgs/install/interfaces_pkg/')

from interfaces_pkg.msg import ControllerState


def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpyFEEDBACK_TOPIC to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal, frame)


# Static folder is not equivalent to the static react build folder.
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
        return "500: Error Serving Page! Check that the React page is properly building."

# API
@app.route('/message_data')
def get_node_data():
    return ros_node.message_data

@app.route('/publish_message')
def get_publish_message():
    ros_node.publish_message()
    return {}

# Feedback
@app.route('/core/feedback')
def get_core_feedback():
    try:
        return {'data': ros_node.message_data[CORE_FEEDBACK]}
    except KeyError:
        return {'data': 'No data was found.'}
    
@app.route('/bio/feedback')
def get_bio_feedback():
    try:
        return {'data': ros_node.message_data[BIO_FEEDBACK]}
    except KeyError:
        return {'data': 'No data was found.'}
    
@app.route('/arm/feedback')
def get_arm_feedback():
    try:
        return {'data': ros_node.message_data[ARM_FEEDBACK]}
    except KeyError:
        return {'data': 'No data was found.'}
    

@app.route('/arm/bio/feedback')
def get_faerie_feedback():
    try:
        return {'humidity': ros_node.message_data[FAERIE_FEEDBACK][-1].humidity, 'temperature': ros_node.message_data[FAERIE_FEEDBACK][-1].temperature}
    except KeyError:
        return {'data': 'No data was found.'}

# Control  
@app.route('/bio/control', methods = ['POST'])
def bio_control():
    if request.method == 'POST':
        command = request.get_json()['command']
        print(f"Sending command {command} to CITADEL")

        if BIO_CONTROL not in ros_node.publishers.keys():
            ros_node.create_string_publisher(BIO_CONTROL)

        ros_node.publish_string_data(BIO_CONTROL, command)

        return {'data': command}
    else:
        print("Invalid method on /bio/control")

@app.route('/arm/bio/control', methods = ['POST'])
def faerie_control():
    if request.method == 'POST':
        command = request.get_json()['command']
        print(f"Sending command {command} to FAERIE")

        if FAERIE_CONTROL not in ros_node.publishers.keys():
            ros_node.create_string_publisher(FAERIE_CONTROL)

        ros_node.publish_string_data(FAERIE_CONTROL, command)

        return {'data': command}
    else:
        print("Invalid method on /arm/bio/control")

@app.route('/arm/control', methods = ['POST'])
def arm_control():
    if request.method == 'POST':
        command = request.get_json()['command']
        print(f"Sending command {command} to the Arm")

        if ARM_COMMAND not in ros_node.publishers.keys():
            ros_node.create_string_publisher(ARM_COMMAND)

        ros_node.publish_string_data(ARM_COMMAND, command)

        return {'data': command}
    else:
        print("Invalid method on /arm/control")

@app.route('/auto/control', methods = ['POST'])
def auto_control():
    if request.method == 'POST':
        command = request.get_json()['command']
        print(f"Sending command {command} to the autonomy node")

        if not ros_node.actions_started:
            return {'data': ""}

        if command == "Stop":
            ros_node.send_autonomy_goal(0, 0, 0, 1)
        elif command == "GoTo":
            ros_node.send_autonomy_goal(1, request.get_json()['lat'], request.get_json()['long'], request.get_json()['period'])
        elif command == "ARUCO":
            ros_node.send_autonomy_goal(2, request.get_json()['lat'], request.get_json()['long'], request.get_json()['period'])
        elif command == "Object":
            ros_node.send_autonomy_goal(3, request.get_json()['lat'], request.get_json()['long'], request.get_json()['period'])

        return {'data': command}
    else:
        print("Invalid method on /auto/control")

# Miscellaneous Endpoints
@app.route('/core/ping')
def ping():
    if not ros_node.services_started:
        return {'data': ""}
    start = time.time()
    response = ros_node.send_ping()
    if response:
        end = time.time()
        return {'data': end - start}
    else:
        return {'data': ""}


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
    # Handle disconnections for the ROS node (image subscribers)
    ros_node.handle_disconnect(request.sid)

# Controller connection handler
@socketio.on(CORE_CONTROL)
def core_control_handling(ly, ry):
    # If the publisher does not already exist, create it
    if CORE_CONTROL not in ros_node.publishers.keys():
        ros_node.create_string_publisher(CORE_CONTROL)
    # Handle actually publishing the data when the publisher exists
    
    ros_node.publish_string_data(CORE_CONTROL, f"ctrl,{ly:0.2f},{ry:0.2f}")

@socketio.on(ARM_CONTROL)
def arm_control_handling(lh, lv, rh, rv, du, dd, dl, dr, 
                         b, a, y, x, l, r, zl, zr, select, start):
    #  If the publisher does not already exist, create it
    if ARM_CONTROL not in ros_node.publishers.keys():
        ros_node.publishers[ARM_CONTROL] = ros_node.create_publisher(ControllerState, ARM_CONTROL, 0)

    # can't send floats over the socket for whatever reason, have to round them on the front end and divide by 100 
    
    # Make use of ControllerState custom interface
    msg = ControllerState()
    msg.lb = l
    msg.rb = r
    msg.plus = start
    msg.minus = select
    msg.ls_x = lh / 100
    msg.ls_y = lv / 100
    msg.rs_x = rh / 100
    msg.rs_y = rv / 100
    msg.a = a
    msg.b = b
    msg.x = x
    msg.y = y
    msg.d_up = du
    msg.d_down = dd
    msg.d_left = dl
    msg.d_right = dr
    msg.lt = zl / 100
    msg.rt = zr / 100

    # Handle data publishing
    ros_node.publishers[ARM_CONTROL].publish(msg)
    print(f"Publishing data to {ARM_CONTROL}: {msg.lt} {msg.rt} {msg.lb} {msg.rb} {msg.plus} {msg.minus} {msg.ls_x} {msg.ls_y} {msg.rs_x} {msg.rs_y} {msg.a} {msg.b} {msg.x} {msg.y} {msg.d_up} {msg.d_down} {msg.d_left} {msg.d_right}")

# Handle image subscription request
@socketio.on('image_subscription')
def image_subscription_message(topic_name):
    req_id = request.sid
    handle_image_subscription(req_id, topic_name)

def handle_image_subscription(socket_id, topic_name):
    print(f'Received request to subscribe to {topic_name} for sensor_msgs.msg.CompressedImage')
    def socket_callback(msg):
        # Do some handling for the image to be sent over to
        # the front-end and displayed in an image in base64
        print(f"Message recieved for image topic \"{topic_name}\"")

        # Convert the message to binary encoding

        # Convert the sensor_msgs.msg.CompressedImage to
        # a CV2 image using the ROS2 CV Bridge
        current_frame = ros_node.opencv_bridge.compressed_imgmsg_to_cv2(msg)
        # Encode the CV2 frame data into JPG image encoding
        frame_data = cv2.imencode('.jpg', current_frame)[1]
        # Convert frame data into a Python bytearray with NumPy
        image_bytes = np.array(frame_data).tobytes()

        # Convert bytearray to base64, and decode the base64 bytearray into a string
        base64_str = base64.b64encode(image_bytes).decode()
        # Emit (broadcast) the data to the websockets
        socketio.emit(topic_name,{'data': base64_str})
        
    ros_node.create_image_subscriber(socket_callback, topic_name, socket_id)
    