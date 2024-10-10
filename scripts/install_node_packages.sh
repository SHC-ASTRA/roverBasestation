#!/bin/env bash

# Make use of yarn to install all necessary
# packages inside of the dynamic React frontend

source /opt/ros/humble/setup.bash
cd react-app
yarn install
