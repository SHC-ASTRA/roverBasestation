#!/bin/env bash

# To be run from the main repository directory

# Source ROS2 dependency
source /opt/ros/humble/setup.bash

cd react-app
# Make use of yarn to install all necessary
# packages inside of the dynamic React frontend
yarn install
# Create a statically build production optimized version
# of the react front-end
yarn build

# Move to the server directory
cd ../server
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