#!/bin/bash

HOME_DIR=/home/ros
export NVM_DIR="$HOME_DIR/.nvm"

# Install Requirement pkg
apt update
apt install -y build-essential cmake ripgrep xsel fuse3 cargo zip curl git

# installs NVM (Node Version Manager)
mkdir -p $NVM_DIR
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
cat <<EOT >>$HOME_DIR/.bashrc
export NVM_DIR="/home/ros/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
EOT

\. "$NVM_DIR/nvm.sh"

# download and install Node.js
nvm install 24

# verifies the right Node.js version is in the environment
node -v # should print `v24.x.x`

# verifies the right NPM version is in the environment
npm -v # should print `11.x.x`

# Install Neovim (Extract mode to avoid FUSE requirement in Docker)
orig_path=$(pwd)
mkdir -p $HOME_DIR/Apps/nvim && nvim_dir=$_ && cd $nvim_dir

# Detect Architecture
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    NVIM_APPIMAGE="nvim-linux-arm64.appimage"
else
    NVIM_APPIMAGE="nvim-linux-x86_64.appimage"
fi

curl -LO https://github.com/neovim/neovim/releases/latest/download/$NVIM_APPIMAGE
chmod u+x ./$NVIM_APPIMAGE

# Do not execute directly (avoids FUSE error), extract it instead
./$NVIM_APPIMAGE --appimage-extract
ln -s $nvim_dir/squashfs-root/usr/bin/nvim /usr/bin/nvim

cd $orig_path

git clone https://github.com/atomon/astronvim_v5.git $HOME_DIR/.config/nvim

echo "alias v='nvim'" >>$HOME_DIR/.bash_aliases