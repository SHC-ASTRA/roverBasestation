# ASTRA Base Station
Welcome! This is the repository for UAH's ASTRA Base Station for the University Rover Challenge in 2024.
ASTRA is a project under the AutoSat branch of Space Hardware Club as part of the University of Alabama in Huntsville.

## Running the Base Station
### Dependencies
* Docker Engine
* Network Connection

### React & ROS Node Basestation
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

## Developers
For anyone adding to this repository, please add your name to the README before making a pull request.
- Jamie Roberson
- Nathan Estep
- Emann Rivero
- Alexander Resurreccion