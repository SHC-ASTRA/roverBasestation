# Performing Development Builds

> [!NOTE]
> All scripts are intended to be run from the parent directory of the rover-Basestation-Release repository. Running these scripts from within another directory will produce incorrect results or outright errors.

## Submodule Updates

To build the submodules, first ensure that the submodule repositories are at their latest versions by running the following commands.

```bash
git submodule init
git submodule update
```

## Complete Rebuild

In order to perform a complete (re)build of the project, run the following script / command.

```bash
. scripts/fullstack_development_build.sh
```

## Front-End Rebuild

> [!NOTE]
> The front-end cannot be "served" as a page by itself. It is necessary for the full application to be run using the below `basestation_initialize.sh` script.

In order to build only the front-end React page, run the following script / command

```bash
. scripts/front_end_build.sh
```

## Back-End Rebuild

In order to build only the back-end ROS2 package, run the following script / command

```bash
. scripts/back_end_build.sh
```
