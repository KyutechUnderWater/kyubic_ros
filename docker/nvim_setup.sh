#!/bin/bash

HOME_DIR=/home/ros

# Install Requirement pkg
sudo apt update && sudo apt install -y build-essential cmake ripgrep xsel fuse3 cargo zip git

# installs NVM (Node Version Manager)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
\. "$HOME/.nvm/nvm.sh"

# download and install Node.js
nvm install 24

# verifies the right Node.js version is in the environment
node -v # should print `v24.1.0`

# verifies the right NPM version is in the environment
npm -v # should print `11.3.0`

# Install Neovim
orig_path=$(pwd)
mkdir -p $HOME_DIR/Apps/nvim && nvim_dir=$_ && cd $nvim_dir
curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.appimage
sudo chmod u+x ./nvim-linux-x86_64.appimage && $_ --version
if [[ $? == 0 ]]; then
	sudo ln -s $nvim_dir/nvim-linux-x86_64.appimage /usr/bin/nvim
else
	./nvim-linux-x86_64.appimage --appimage-extract >&/dev/null && sudo ln -s $nvim_dir/squashfs-root/usr/bin/nvim /usr/bin/nvim
fi
cd $orig_path

git clone https://github.com/atomon/astronvim_v5.git $HOME_DIR/.config/nvim

echo "alias v='nvim'" >>$HOME_DIR/.bash_aliases
