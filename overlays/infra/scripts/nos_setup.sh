# !/usr/bin/env bash


git config --global user.name "baseonballs: Jeffrey Lucas"
git config --global user.email "lucasjt@gmail.com"


# install the pnpm package manager
# and the fnm node version manager
curl -fsSL https://get.pnpm.io/install.sh | sh -

curl -fsSL https://fnm.vercel.app/install | bash
eval "$(fnm env --use-on-cd)"

# install the latest LTS version of node
fnm install --lts

# and the python development tools
sudo apt update
sudo apt install -y python3 python3-pip python3-venv python3-dev build-essential make cmake
