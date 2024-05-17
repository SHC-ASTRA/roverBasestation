#!/bin/bash
source /opt/ros/humble/setup.bash
cd ros_msgs
colcon build
source install/setup.bash
cd ../../react-app
yarn build
cd ../server
python3 -m flask run --no-reload --host=0.0.0.0