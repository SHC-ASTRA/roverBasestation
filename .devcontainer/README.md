# VSCode Dev Container

This file contains the configuration for the VSCode development containers.

<https://code.visualstudio.com/docs/devcontainers/containers>

- `Dockerfile` contains the docker configuration for use inside the container.
- `devcontainer.json` contains the VSCode configuration for inside
the container.
- `.bashrc` is the bash startup file. A symlink gets placed in the home
  directory that points here.
- `starship.toml` is the configuration for our prompt.

Basestation uses exclusively container development. You can technically get away
with bare metal development by following the commands in the Dockerfile on
Ubuntu Jammy Jellyfish, but why would you do that?

Windows users should either use a Linux virtual machine or WSL. Do not use
Docker Engine directly on Windows! It is significantly slower than any other option.

If you haven't already, check out [the main read me](../README.md) for
instructions on setting up the development environment.
