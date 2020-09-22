#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd $MY_PATH/../../utils/tmuxinator

echo "$0: installing tmuxinator"

sudo apt-get -y install ruby gem

gem build tmuxinator.gemspec
sudo gem install tmuxinator -v 1.1.5
