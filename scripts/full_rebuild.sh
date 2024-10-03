#!/bin/env bash

last_path=$(pwd)

. scripts/install_node_packages.sh
cd $last_path
. scripts/build_ros_interface.sh
cd $last_path
. scripts/build_yarn.sh
cd $last_path
. scripts/set_up_python.sh
cd $last_path
. scripts/launch.sh
cd $last_path
