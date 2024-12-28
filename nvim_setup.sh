#!/bin/bash

# Install Requirement pkg
sudo apt update && sudo apt install -y build-essential cmake ripgrep xsel fuse3 cargo zip git

# installs NVM (Node Version Manager)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
eval "$(cat ~/.bashrc | tail -n +10)"

# download and install Node.js
nvm install 20

# verifies the right Node.js version is in the environment
node -v # should print `v20.12.2`

# verifies the right NPM version is in the environment
npm -v # should print `10.5.0`


# Install Neovim
orig_path=$(pwd)
mkdir -p ~/Apps/nvim && nvim_dir=$_ && cd $nvim_dir
curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim.appimage
sudo chmod u+x ./nvim.appimage && $_ --version
if [[ $? == 0 ]]; then
	ln -s $nvim_dir/nvim.appimage /usr/bin/nvim
else
	./nvim.appimage --appimage-extract >& /dev/null && sudo ln -s $nvim_dir/squashfs-root/usr/bin/nvim /usr/bin/nvim
fi
cd $orig_path

git clone https://github.com/atomon/astronvim_config_v4.git ~/.config/nvim

echo "alias v='nvim'" >> ~/.bash_aliases
eval "$(cat ~/.bashrc | tail -n +10)"
