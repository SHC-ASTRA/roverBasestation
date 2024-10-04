#!/bin/env bash

cd server
source /opt/ros/humble/setup.bash
source ros_msgs/install/setup.bash
source .flaskenv
python3 -m flask run --no-reload --host=0.0.0.0 --port=80
