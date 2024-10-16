#!/bin/env bash

# To be run from the main repository directory

# Source ROS2 dependency
source /opt/ros/humble/setup.bash

# Move to the server directory
cd server
# Source the Python virtual environment
source .flaskenv
# Move to the ros_msgs ROS2 package
cd ros_msgs
# Build the ROS2 interfaces package
# TODO: to be migrated to a submodule repository
# rather than its current location
colcon build
# Move back to the server directory
cd ..
# Ensure the back-end python dependencies are installed
pip install -r requirements.txt
# Return to the main directory
cd ..