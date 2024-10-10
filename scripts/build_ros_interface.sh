#!/bin/env bash

# Build the ROS2 interfaces package

# TODO: to be migrated to a submodule repository
# rather than its current location

source /opt/ros/humble/setup.bash
cd server/ros_msgs
colcon build