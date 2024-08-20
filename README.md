# ASTRA Base Station

Welcome! This is the repository for UAH's ASTRA Base Station for the University
Rover Challenge in 2024.

ASTRA is a project under the AutoSat branch of Space Hardware Club as part of
the University of Alabama in Huntsville.

## Development Platform Recommendations

Primary development is being done on Ubuntu 22.04.3 LTS, if you intend to fork
or continue development this version of Ubuntu or newer is recommended.

It is possible to perform development on any device that supports Docker Engine
through Docker Desktop or CLI interface. It is recommended to use an Ubuntu
virtual machine as the `Dockerfile` may take extended amounts of time to rebuild
for each change. This can reduce the amount of time spent doing actual
development (though [some developers may prefer this](https://xkcd.com/303/)).

If you are unable to create an Ubuntu virtual machine, it is recommended to make
use of a Google Cloud Computer or an Akamai Linode with the minumum
specifications as listed:

* 2 virtual cores, 4 recommended
* 2Gb of RAM, 4Gb recommended
* 20Gb of total storage, 30Gb recommended

## Running the Base Station

### Dependencies

* Docker Engine ([Install](https://docs.docker.com/engine/install/ubuntu/))
* Network Connection

### Running Docker

It is first important that you have the Docker commandline engine installed. The
installation process can be found [here](https://docs.docker.com/engine/install/).
It is recommended to install on MacOS or Linux, though Windows may work too.

It is then possible to clone this repository. SSH is recommended if you intend
to make any changes. The documentation for how to add an SSH key to your Github
account can be found [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

```bash
# Clone
git clone git@github.com:SHC-ASTRA/rover-Basestation.git
# Set up submodules
git submodule init
git submodule update
```

After cloning the repository the following can be run

```bash
# Change directories into the repo
cd rover-Basestation-Release
# Start the docker container
sudo docker-compose up
```

## Non-Docker Development

### Development Dependencies

* **NodeJS** v18.18.2 (Recommended install with [Node Version Manager](https://github.com/nvm-sh/nvm#installing-and-updating))
    * `corepack` npm package for use of Yarn (Recommended)
* **ROS2 Humble** ([Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
* **APT Packages**

    * APT packages will change as the project develops. It is recommended to
    follow the `apt install` commands in `Dockerfile` to find the most
    up-to-date package list.

### Running the Basestation without Docker

The basestation can be run outside of the Docker image by similarly following
the docker `RUN` commands.

## Contributors

For anyone adding to this repository, please add your name to the README before
making a pull request.

- Jamie Roberson
- Alexander Resurreccion
- Anshika Sinha
- Riley McLain
