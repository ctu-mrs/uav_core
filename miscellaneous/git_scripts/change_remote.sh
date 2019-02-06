#!/bin/bash

USERNAME="git"
ADDRESS="mrs.felk.cvut.cz"

#############################################

# MY_PATH=`dirname "$0"`
# MY_PATH=`( cd "$MY_PATH" && pwd )`

MY_PATH="./"

# parse the .gitmodules files in the PATH
if [ -f "$MY_PATH/$1/.gitmodules" ]; then

  # find each module in the .gitmodules file and extract its relative path from the doublequotes
  SUBMODULES=($( cat "$MY_PATH/$1/.gitmodules" | grep "^\[submodule" | cut -d "\"" -f2 | cut -d "\"" -f1 ))

  cp $MY_PATH/$1/.gitmodules $MY_PATH/$1/.gitmodules_new

  # for each submodule
  for submodule in $SUBMODULES; do

    # recursively find its submodules
    if [ -z "$1" ]; then # if we are in the root repo
      repo_to_local "$submodule"
    else
      repo_to_local "$1/$submodule"
    fi

    # extract the name in the superrepo
    echo SUBMODULE: $submodule
    REPO_NAME=$( echo $submodule | sed -r 's/.*\/([^\/]+)/\1/g' )
    echo REPO_NAME: $REPO_NAME

    # extract the submodule server path
    CMD="cat '$MY_PATH/$1/.gitmodules' | grep -e 'url.*$REPO_NAME' | sed -r 's/.*:(.*)$REPO_NAME.*/\1/g'"
    SUB_PATH=$( eval $CMD )
    echo SUB_PATH: $SUB_PATH

    # change the remote
    git config --file=.gitmodules_new "submodule.$submodule.url" "$USERNAME@$ADDRESS:$SUB_PATH/$REPO_NAME"
    git submodule sync > /dev/null
    cd "$MY_PATH"

  done

  mv "$MY_PATH/$1/.gitmodules_new" "$MY_PATH/$1/.gitmodules"

fi

# fix the super repo
if [ -z "$1" ]; then

  # extract the name in the superrepo
  CMD="git remote -v | grep origin | head -n 1 | cut -d ":" -f2 | sed -r 's/.*\/(.+)\s.*$/\1/g'"
  REPO_NAME=$( eval $CMD )
  echo REPO_NAME: $REPO_NAME

  CMD="git remote -v | grep origin | head -n 1 | cut -d ":" -f2 | sed -r 's/(.*)$REPO_NAME.*/\1/g'"
  SUB_PATH=$( eval $CMD )
  echo SUB_PATH: $SUB_PATH

  # push the local repo
  git add .gitmodules
  git commit -m "switched .gitmodules to local"
  git push

fi
