#!/bin/env bash

cd server
# Source the standard ROS2 library of functions
source /opt/ros/humble/setup.bash
# Source the (to be submodularized) ROS2 interace package
source ros_msgs/install/setup.bash
# Source the Python virtual environment
source .flaskenv
python3 -m flask run --no-reload --host=0.0.0.0 --port=80
