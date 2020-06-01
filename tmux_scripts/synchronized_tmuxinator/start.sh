#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

# start tmuxinator
tmuxinator
