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

RUN source /opt/ros/humble/setup.bash \
    && cd server \
    && yarn install
RUN source /opt/ros/humble/setup.bash \
    && cd react-app \
    && yarn install

#################
# ROS Building
#################

# Build the ROS interface
RUN source /opt/ros/humble/setup.bash \
    && cd server/ros_msgs \
    && colcon build

# Compile the interface to Javascript
# https://github.com/RobotWebTools/rclnodejs-cli/tree/develop/message-generator-tool
RUN source /opt/ros/humble/setup.bash \
    && source server/ros_msgs/install/setup.bash \
    && cd react-app \
    && yarn build

# set up the python environment
RUN cd server && pip install -r requirements.txt

CMD ["/bin/bash", "-c", "cd server && source /opt/ros/humble/setup.bash && source ros_msgs/install/setup.bash && python3 -m flask run --no-reload --host=0.0.0.0 --port=80"]
