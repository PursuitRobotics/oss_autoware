# Build Environment for Pursuit Robotics Autoware Stack

This is an extension of the Autoware Foundation build system.  The bootstrapping of autowawre is performed from within the overlays directory of the autoware foundation root source

## Building Sources from Native Operating System (NOS).

Building on the native operating system otherwise known as NOS. It will install frameworks, toolskits, drivers, and depdent packages directly into your OS stack.

### Prerequisition

1. Ubuntu 22.04 (Jammy)
2. git
3. python3
4. docker & docker-compose
5. pnpm (fast and more efficient replacement for npm)
6. fnm (fast node manager)

Note refer to ../overlays/scripts/nos_setup.sh to install prerequisites global environment variables and tools mentioned above. These will need to on your system prior to using the autoware custom build system.

## Building from Sources

### Step 1 - Setup the Dev Environment

```bash
    git clone https://github.com/PursuitRobotics/oss_autoware.git
    cd oss_autoware

    # git will can find the root git repo project by using --show-toplevel flag
    local ROOT_DIR=git rev-parse --show-toplevel
    local WORKSPACE=${ROOT_DIR}

    cd ${WORKSPACE}/overlays

    # to displa the list of make-takks use the --help
    make --help

    # use the autoware setup-dev scripts to install the prerequistes and requirement tools, and packages used by python, cmake, c/c++ etc.  This step is required for the initial staging of the autowave build sources.
    make nos-setup-dev-no-cuda
```

### Step 2 - Create the Autoware Workspace

```bash
    local ROOT_DIR=git rev-parse --show-toplevel
    local WORKSPACE=${ROOT_DIR}

    cd ${WORKSPACE}/overlays

    # Generate the autoware workspace. The staging area (transient and resources managed by autoware build system (crosdep & olcon)
    make nos-workspace-autoware
```

### Step 3 - Install the ROS2 Dependency Tool (ROSDEP)

```bash
    local ROOT_DIR=git rev-parse --show-toplevel
    local WORKSPACE=${ROOT_DIR}

    cd ${WORKSPACE}/overlays
    make nos-ros-deps
```

### Step 4 - Execute build via colcon build system

```bash
    local ROOT_DIR=git rev-parse --show-toplevel
    local WORKSPACE=${ROOT_DIR}

    cd ${WORKSPACE}/overlays

    # Buil the entire autoare software layers using colcon build system
    # note: rosdep must be invoked or setup first. It provides the dependency injections and pakcage integrations sourced by the *.repos (git and subgit repositories)
    make nos-build
```

### Step 5 - End-to-End Verificaiton

Launch a simple planning simulator. 

```shell
    local ROOT_DIR=git rev-parse --show-toplevel
    local WORKSPACE=${ROOT_DIR}

    cd ${WORKSPACE}/overlays
    make launch-
```