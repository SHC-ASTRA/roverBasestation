# ASTRA Base Station
Welcome! This is the repository for UAH's ASTRA Base Station for the University Rover Challenge in 2024.
ASTRA is a project under the AutoSat branch of Space Hardware Club as part of the University of Alabama in Huntsville.

## Development Platform Recommendations
Primary development is being done on Ubuntu 22.04.3 LTS, if you intend to fork or continue development this version of Ubuntu or newer is recommended.

It is possible to perform development on any device that supports Docker Engine through Docker Desktop or CLI interface. This is not recommended as the `Dockerfile` may take extended amounts of time to rebuild for each change. This can reduce the amount of time spent doing actual development.

If you are unable to obtain a physical Ubuntu device it is recommended to make use of a Google Cloud Computer or an Akamai Linode with the minumum specifications as listed
* 2 virtual cores, 4 **recommended**
* 2Gb of RAM, 4Gb **recommended**
* 20Gb of total storage, 30Gb **recommended**

## Running the Base Station
### Dependencies
* Docker Engine ([Install](https://docs.docker.com/engine/install/ubuntu/))
* Network Connection

### Running Docker
It is first important that you have the Docker commandline engine installed. The installation process can be found [here](https://docs.docker.com/engine/install/). It is recommended to install on Ubuntu Jammy (22.04.3) LTS.

It is then possible to clone this repository. SSH is recommended if you intend to make any changes. The documentation for how to add an SSH key to your Github account can be found [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).
```
git clone git@github.com:SHC-ASTRA/rover-Basestation.git
```
After cloning the repository the following can be run
```bash
# Change directories into the repo's docker folder
cd rover-basestation/basestation-docker
# Build the docker image from the directory
# A network connection is necessary for this portion
docker build -t basestation .
# Run the docker image and expose the necessary port
# The server will be broadcast to all network interfaces
docker run -p 8000:8000 basestation
```

## Non-Docker Development
### Development Dependencies
* NodeJS v18.18.2 (Recommended install with [Node Version Manager](https://github.com/nvm-sh/nvm#installing-and-updating))
    * `corepack` npm package for use of Yarn (Recommended)
* ROS2 Humble ([Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
* APT Packages
    * `software-properties-common`
    * `make`, `build-essential`, `cmake`
    * `python3-colcon-common-extensions` ([Install](https://colcon.readthedocs.io/en/released/user/installation.html))


### Running the Basestation without Docker
The basestation can be run outside of the Docker image by similarly following the docker `RUN` commands. The below makes use of Yarn, which is designed to be faster and easier than NPM.

```bash
# Enable yarn with corepack
corepack enable

# Start inside the root of the repository, move to the main server
cd basestation-docker

# Build the ROS2 service interface package
cd health_interface
source /opt/ros/humble/setup.bash
colcon build
cd ..

# Install Nodejs packages with yarn
# In order to install the server packages, ROS2 must be sourced.
# We do not do it here because it has already been done for the ROS2 package building.
cd server && yarn install && cd ..
cd react-app && yarn install && cd ..

# Make use of the rclnodejs-cli package.
# Convert the interface package to rclnodejs supported Javascript.
source health_interface/install/setup.bash
cd server && yarn rclnodejs-cli generate-ros-messages && cd ..

# Build the React and Typescript ROS2 / HTTP host server
# The server interface implements this to simplify
cd server && yarn build
# Then run the program
yarn prod
```

If you are only performing client-side updates it is possible to rebuild the React while the program is running (in another command terminal). This is possible - without restarting the program and rebuilding the Typescript - with the below command in the `basestation-docker/server` directory.
```bash
yarn build-react
```
Refresh the browser page after running this command, and the UI will update. This is possible because the web-page is served as a static HTML page by an Express HTTP handler. When you refresh the page, Express re-fetches the page from the `basestation-docker/react/build` directory.

After building the ROS2 interface package, compiling it into `rclnodejs` Javascript, it is possible to only worry about sourcing ROS2 packages, and running the program without pre-emptively building the React & Typescript. This can be done with the below command - that builds the Typescript and React - in the `basestation-docker/server` directory.
```bash
source /opt/ros/humble/setup.bash && source ../health_interface/install/setup.bash
yarn dev
```



## Contributors
For anyone adding to this repository, please add your name to the README before making a pull request.
- Jamie Roberson
- Alexander Resurreccion
- Anshika Sinha