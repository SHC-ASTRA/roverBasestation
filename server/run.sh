#!/bin/bash
source /opt/ros/humble/setup.bash
cd ../react-app
yarn build
cd ../server
flask run --no-reload --host=0.0.0.0