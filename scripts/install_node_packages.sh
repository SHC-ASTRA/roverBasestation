#!/bin/env bash

last_path=$(pwd)

source /opt/ros/humble/setup.bash
cd server
yarn install

cd $last_path

source /opt/ros/humble/setup.bash
cd react-app
yarn install
