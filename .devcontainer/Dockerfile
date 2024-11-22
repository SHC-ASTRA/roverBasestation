# using image with ros humble already installed
# https://hub.docker.com/_/ros/
FROM ros:humble-ros-core-jammy

# change shell to bash
SHELL ["/bin/bash", "-c"]

# set up astra user
RUN useradd -m -U astra \
    && usermod -a -G sudo astra \
    && chsh -s /usr/bin/bash astra \
    && passwd -d astra


###########################
# INSTALL SYSTEM PACKAGES #
###########################

RUN apt-get update

# add standard packages
RUN apt-get install -y software-properties-common \
    && add-apt-repository -y universe \
    && apt-get update

# add development tools (this is a dev container)
RUN apt-get install -y \
    curl make build-essential cmake \
    git vim sudo g++

# add colcon
# see: <https://colcon.readthedocs.io/en/released/user/installation.html>
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-colcon-common-extensions

# install python build requirements for pyenv
# see: <https://github.com/pyenv/pyenv/wiki#suggested-build-environment>
RUN apt-get install -y \
    libssl-dev zlib1g-dev libbz2-dev libreadline-dev \
    libsqlite3-dev libncursesw5-dev xz-utils tk-dev libxml2-dev \
    libxmlsec1-dev libffi-dev liblzma-dev


###########################
# SWITCH TO NON-ROOT USER #
###########################

USER astra

# install nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash

# install node and npm
ENV NODE_VERSION=20
RUN source ~/.nvm/nvm.sh \
    && nvm install $NODE_VERSION --latest-npm \
    && nvm alias default $NODE_VERSION \
    && nvm use default

# install pyenv
RUN curl https://pyenv.run | bash \
    && echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc \
    && echo 'eval "$(pyenv init -)"' >> ~/.bashrc
ENV PYENV_ROOT="/home/astra/.pyenv"

# install python
ENV PY_VERSION=3.12
RUN $PYENV_ROOT/bin/pyenv install $PY_VERSION -v \
    && $PYENV_ROOT/bin/pyenv global $PY_VERSION
