#!/bin/bash
# author: Robert Penicka

echo "Installing git Large File Stograge (LFS) support"
sudo apt-get -y install software-properties-common
sudo curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get -y install git-lfs
git lfs install
echo "done installing LFS"
