#!/bin/env bash

# Install the Python dependencies as outlined
# for the Flask backend server

# This DOES NOT include the ROS2 dependencies that should
# be installed in any dev environment

cd server
pip install -r requirements.txt