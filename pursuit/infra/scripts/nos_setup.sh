# !/usr/bin/env bash


# This script sets up the environment for the nos overlay.
# add to ~/.bashrc or ~/.bash_aliasas
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export AW_HOME=/srv/autoware
export AW_WORKSPACE=${AW_HOME}/dev/pursuit/oss_autoware
export AW_DATA=${AW_HOME}/autoware_data
export AW_MAPS=${AW_HOME}/autoware_maps
export AW_LAUNCH=${AW_HOME}/autoware_launch


# make sure /srv/autoware is owned by $USER:$USER
sudo chown ${USER}:${USER}-R ${AW_HOME}

# create a symlink which points ~/autoware to the path /srv/autoware

ln -s ${AW_HOME} ~/autoware

function find_build_errs() {
    # Find all non-empty stderr.log files and store the list in a variable.
    error_files=$(grep -l -v '^$' -r ${AW_WORKSPACE}/log/latest_build --include="stderr.log")

    # Check if the variable is empty.echo 
    if [ -z "$error_files" ]; then
    # If it's empty, no files with errors were found.
    echo "no build errors found"
    else
    # If it's not empty, print a header and the list of files with errors.
    echo "Build errors found in the following files:"
    echo "$error_files"
    # You might want to exit with an error code to stop the script.
    # exit 1
    fi
}

function setup_ros2 {
    local DIR=${HOME}/dev/pursuit/oss_autoware

    export AW_BUILD=$DIR/build
    export AW_LOGS=$DIR/log
    export AW_INSTALL=$DIR/install
}

function setup_dirs() {
    sudo mkdir -p ${AW_DATA}
    sudo mkdir -p ${AW_MAPS}
    
    sudo chown ${USER}:${USER} ${AW_DATA}
    sudo chown ${USER}:${USER} ${AW_MAPS}
}

git config --global user.name "baseonballs: Jeffrey Lucas"
git config --global user.email "lucasjt@gmail.com"


# install the pnpm package manager
# and the fnm node version manager
curl -fsSL https://get.pnpm.io/install.sh | sh -

curl -fsSL https://fnm.vercel.app/install | bash
eval "$(fnm env --use-on-cd)"

# install the latest LTS version of node
fnm install --lts

# and the python development tools - these tools are handled by the ./setup-dev-env.sh script
#sudo apt update
#sudo apt install -y python3 python3-pip python3-venv python3-dev build-essential make cmake
