# !/usr/bin/env bash


# This script sets up the environment for the nos overlay.
# add to ~/.bashrc or ~/.bash_aliasas
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export AW_DATA=/srv/autoware/autoware_data
export AW_MAPS=/srv/autoware/autoware_maps

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
