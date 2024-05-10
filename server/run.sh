#!/bin/bash
source /opt/ros/humble/setup.bash
cd ../react-app
yarn build
cd ../server
python3 -m flask run --no-reload
