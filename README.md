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

## Dependencies

* Visual Studio Code ([Install](https://code.visualstudio.com/download))
* Github CLI ([Install](https://cli.github.com/))

## Running the Base Station


### Windows

Open "Windows Features' and enable "Windows Subsystem for Linux".

Install [Visual Studio Code](https://code.visualstudio.com/download).

In a terminal run
```bash
wsl --install
```

Follow the instructions on screen, creating an account for the ubuntu vm.
Proceed with linux install instructions.

### Linux

It is first important that you have the Docker commandline engine installed. The
installation process can be found [here](https://docs.docker.com/engine/install/).

Install VSCode:
```bash
sudo snap install code --classic
```

Generate an SSH key:
```bash
ssh-keygen -t ed25519 -C "email@example.com"  -f ~/.ssh/github
```

Tell SSH to use your key for github.com:
```bash
cat << EOF >> ~/.ssh/config 
Host github.com
        IdentityFile ~/.ssh/github
EOF
```

[Install Github CLI](https://github.com/cli/cli/blob/trunk/docs/install_linux.md#debian-ubuntu-linux-raspberry-pi-os-apt).

Login to Github CLI:
```bash
gh auth login
GitHub.com
SSH
/home/[username]/.ssh/github.pub
GitHub CLI
Login with a web browser
```

Proceed to clone this repository.
```bash
# Clone
gh repo clone SHC-ASTRA/rover-Basestation-Release

# Change directories into the repo
cd rover-Basestation-Release

# Set up submodules
git submodule init
git submodule update
```

Open the repository with VSCode:
```bash
code .
```

VSCode will suggest numerous plugins to install. Once you have installed the plugins, press
`Control + Shift + P` and type `reopen in dev container`.

After connecting to the dev container, run the following command in a new terminal:
```bash
. scripts/full_rebuild.sh
```
and press 'y' as necessary. Once all dependencies have finished installing, the web server
should start and visible on localhost.


## Troubleshooting
	
If you get an error related to push permissions, it's possible that your SSH key
was not properly registered to the SSH agent. Run the following commands to fix this:

Add your local user to the docker group:
```bash
sudo usermod -aG docker $USER
exec newgrp docker
```

Bind github to the ssh agent:
```bash
	cat << EOF >> ~/.bashrc
	eval \$(ssh-agent) > /dev/null
	ssh-add -q ~/.ssh/github
	EOF

	exec bash

	pkill "code"
	code .
```

## Contributors

For anyone adding to this repository, please add your name to the README before
making a pull request.

- Jamie Roberson
- Alexander Resurreccion
- Anshika Sinha
- Riley McLain
- Roald Schaum
