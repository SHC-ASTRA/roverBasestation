# ASTRA's custom docker image with all of the build dependencies installed
FROM ghcr.io/shc-astra/rover-basestation-docker@sha256:c916b6da52871ab78b33075477e40e5c16de81dc5433e80ad951640bbd286630

# change shell to bash
SHELL ["/bin/bash", "-c"]

###############
# WORK FILES
###############

WORKDIR /app
COPY ./ /app

##########################
# Install node packages
##########################

RUN bash scripts/install_node_packages.sh

#################
# ROS Building
#################

# Build the ROS interface
RUN bash sripts/build_ros_interface.sh

# Compile the interface to Javascript
# https://github.com/RobotWebTools/rclnodejs-cli/tree/develop/message-generator-tool
RUN bash scripts/build_yarn.sh

# set up the python environment
RUN bash scripts/set_up_python.sh

CMD bash scripts/launch.sh
