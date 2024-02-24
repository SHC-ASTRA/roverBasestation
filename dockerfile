# ROS image, ros-humble-ros-core package installed
# on Ubuntu Jammy
# https://hub.docker.com/_/ros/
FROM ros:humble-ros-core-jammy

# Make use of bash, not the system shell
RUN mv /bin/sh /bin/sh.bak && ln -s /bin/bash /bin/sh

######################
# WORK FILES
######################

WORKDIR $HOME
COPY ./ ./
# Remove all possibly copied node modules
RUN rm -rf node_modules server/node_modules react-app/node_modules

######################
# ROS2 Humble Install
######################

# Using the ROS docker image
# It is no longer necessary to install ROS2
# or source.
# The ROS2 image makes use of an entrypoint script

# See the dockerfile here:
# https://github.com/osrf/docker_images/tree/20e3ba685bb353a3c00be9ba01c1b7a6823c9472/ros/humble/ubuntu/jammy 

#####################
# Packages install
#####################

# Update packages repositories
RUN apt update
# Add standard packages
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe
# Update the repositories again
RUN apt update

#####################
# Install packages
#####################

# Add standard packages
RUN add-apt-repository -y universe
# Update packages repositories
RUN apt update
# Repository keys & certificates
RUN apt install -y ca-certificates gnupg
RUN mkdir -p /etc/apt/keyrings
# common software properties, curl, make, build-essentials, cmake
RUN apt install -y software-properties-common curl make build-essential cmake

# Add colcon repository
# https://colcon.readthedocs.io/en/released/user/installation.html
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Add nodejs repository
# Specifically the node_18.x repo, for the supported rclnodejs version
RUN echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_18.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list
# Add nodejs repo key
RUN curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg

########################
# Update repositories
########################
 
RUN apt update

# Install colcon
RUN apt install -y python3-colcon-common-extensions

# Install nodejs
RUN apt install -y nodejs

# Install corepack
RUN npm install -g corepack
# Enable yarn
RUN corepack enable

##########################
# Install node packages
##########################

RUN cd server && source /opt/ros/humble/setup.bash && yarn install
RUN cd react-app && yarn install

#################
# ROS Building
#################

# Build the ROS interface
RUN cd health_interface && source /opt/ros/humble/setup.bash && colcon build
# Compile the interface to Javascript
# https://github.com/RobotWebTools/rclnodejs-cli/tree/develop/message-generator-tool
RUN cd server && source /opt/ros/humble/setup.bash && source ../health_interface/install/setup.bash && yarn rclnodejs-cli generate-ros-messages

#################
# Final builds
#################

# Build the Typescript & React
RUN cd server && source /opt/ros/humble/setup.bash && yarn build
# Source the health_interface and start the server
CMD ["/bin/bash", "-c", "cd server && source /opt/ros/humble/setup.bash && source ../health_interface/install/setup.bash && yarn prod"]