#!/bin/bash

USERNAME="git"
ADDRESS="gitnuc"
SUBFOLDER=""

#############################################

if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
  VIM_BIN="$(whereis nvim | awk '{print $2}')"
  HEADLESS="--headless"
elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
  VIM_BIN="$(whereis vim | awk '{print $2}')"
  HEADLESS=""
fi

# path to this script
SCRIPT_PATH=`realpath "$0"`

# path to work on
if [ -z $1 ]; then
  MY_PATH="./"
else
  MY_PATH=$1
fi

MY_PATH=`( cd "$MY_PATH" && pwd )`

# parse the .gitmodules files in the PATH
if [ -f "$MY_PATH/.gitmodules" ]; then

  # find each module in the .gitmodules file and extract its relative path from the doublequotes
  SUBMODULES=$( cat "$MY_PATH/.gitmodules" | grep "^\[submodule" | cut -d "\"" -f2 | cut -d "\"" -f1 )

  cp $MY_PATH/.gitmodules $MY_PATH/.gitmodules_new

  # for each submodule
  for submodule in $SUBMODULES; do

    echo ""

    # extract the name in the superrepo
    echo SUBMODULE: $submodule

    REPO_NAME=$( echo $submodule | sed -r 's/.*\/([^\/]+)/\1/g' )
    echo REPO_NAME: $REPO_NAME

    # recursively find its submodules
    CMD="$0 $MY_PATH/$submodule"
    echo "RECURSING IN"
    eval $CMD
    echo "RECURSING OUT"

    # extract the submodule server path
    CMD="cat "$MY_PATH/.gitmodules" | vims '%g/submodule.*$REPO_NAME.]/norm jjddggPjdG' | vims ':norm \$F/bd0' | vims '%s/\/\///g' | vims ':norm \$F/D'"
    echo "$CMD"
    SUB_PATH=$( eval $CMD )
    echo "SUB_PATH: '$SUB_PATH'"

    # check if the repo was actually created
    CMD="ssh $USERNAME@$ADDRESS 'test -d ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME'"
    eval $CMD
    RET=$?
    if [[ "$RET" == "0" ]]; then
      echo "Cannot create the repo, already exists...\n"
    else
      # create the bare repo
      CMD="ssh $USERNAME@$ADDRESS 'mkdir -p ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME; cd ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME; git init --bare'"
      eval $CMD

      # check if the repo was actually created
      CMD="ssh $USERNAME@$ADDRESS 'test -d ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME'"
      eval $CMD
      RET=$?
      if [[ "$RET" != "0" ]]; then
        echo "Cannot create the repo, quitting...\n"
        exit
      fi
    fi

    # push the local repo
    cd "$MY_PATH/$submodule"
    git remote remove local
    git remote add local "$USERNAME@$ADDRESS:~/$SUBFOLDER/$SUB_PATH/$REPO_NAME"
    git push --all local -u
    cd "$MY_PATH"
    git config --file=.gitmodules_new "submodule.$submodule.url" "$USERNAME@$ADDRESS:~/$SUBFOLDER/$SUB_PATH/$REPO_NAME"
    git submodule sync > /dev/null

  done

  mv "$MY_PATH/.gitmodules_new" "$MY_PATH/.gitmodules"

fi

if [ -f "$MY_PATH/.gitman.yml" ]; then

  # find each module in the .gitmodules file and extract its relative path from the doublequotes
  SUBMODULES=$( cat "$MY_PATH/.gitman.yml" | grep "name:" | vims -l '0df:x' )

  cp $MY_PATH/.gitman.yml $MY_PATH/.gitman.yml_backup

  gitman update

  # for each submodule
  for submodule in $SUBMODULES; do

    echo ""

    # extract the name in the superrepo
    echo SUBMODULE: $submodule
    REPO_NAME=$submodule

    CMD="cat "$MY_PATH/.gitman.yml" | vims '%g/name: $REPO_NAME/norm /link:ddggPjdG' | vims ':norm 0df:x'"
    SUBMODULE_LOC=$( eval $CMD )
    echo SUBMODULE_LOC: $SUBMODULE_LOC

    # recursively find its submodules
    CMD="$0 $MY_PATH/$SUBMODULE_LOC"
    echo "RECURSING IN"
    eval $CMD
    echo "RECURSING OUT"

    # extract the submodule server path
    CMD="cat "$MY_PATH/.gitman.yml" | vims '%g/name: $REPO_NAME/norm kddggPjdG' | vims ':norm 0df:x' | vims ':norm \$F/F-bd0' | vims '%s/\/\///g' | vims ':norm \$F/D'"
    echo "$CMD"
    SUB_PATH=$( eval $CMD )
    echo "SUB_PATH: '$SUB_PATH'"

    # check if the repo was actually created
    CMD="ssh $USERNAME@$ADDRESS 'test -d ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME'"
    eval $CMD
    RET=$?
    if [[ "$RET" == "0" ]]; then
      echo "Cannot create the repo, already exists...\n"
    else
      # create the bare repo
      CMD="ssh $USERNAME@$ADDRESS 'mkdir -p ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME; cd ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME; git init --bare'"
      eval $CMD

      # check if the repo was actually created
      CMD="ssh $USERNAME@$ADDRESS 'test -d ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME'"
      eval $CMD
      RET=$?
      if [[ "$RET" != "0" ]]; then
        echo "Cannot create the repo, quitting...\n"
        exit
      fi
    fi

    # push the local repo
    cd "$MY_PATH/$SUBMODULE_LOC"
    git remote remove local
    git remote add local "$USERNAME@$ADDRESS:~/$SUBFOLDER/$SUB_PATH/$REPO_NAME"
    git push --all local -u
    cd "$MY_PATH"
    CMD="$VIM_BIN $HEADLESS -u "~/git/linux-setup/submodules/profile_manager/epigen/epigen.vimrc" -E -s -c '%g/name: $REPO_NAME/norm k0WWWC$USERNAME@$ADDRESS:~/$SUBFOLDER/$SUB_PATH/$REPO_NAME' -c 'wqa' -- $MY_PATH/.gitman.yml"
    echo $CMD
    eval $CMD

  done

  rm "$MY_PATH/.gitman.yml_backup"

  gitman lock

fi

# fix the super repo
if [ -f "$MY_PATH/.gitmodules" ] || [ -f "$MY_PATH/.gitman.yml" ]; then

  # extract the name in the superrepo
  CMD="git remote -v | grep origin | head -n 1 | cut -d ":" -f2 | sed -r 's/.*\/(.+)\s.*$/\1/g'"
  REPO_NAME=$( eval $CMD )
  echo REPO_NAME: $REPO_NAME

  CMD="git remote -v | grep origin | head -n 1 | vims ':norm \$F/F-bd0' | vims '%s/\/\///g' | vims ':norm \$F/D'"
  echo $CMD
  SUB_PATH=$(eval $CMD)
  echo SUB_PATH: $SUB_PATH

  # check if the repo already exists
  CMD="ssh $USERNAME@$ADDRESS 'test -d ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME'"
  echo $CMD
  eval $CMD
  RET=$?
  if [[ "$RET" == "0" ]]; then
    echo "Cannot create the repo, already exists..."
    return
  fi

  # create the bare repo
  CMD="ssh $USERNAME@$ADDRESS 'mkdir -p ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME; cd ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME; git init --bare'"
  echo $CMD
  eval "$CMD"

  # check if the repo was actually created
  CMD="ssh $USERNAME@$ADDRESS 'test -d ~/$SUBFOLDER/$SUB_PATH/$REPO_NAME'"
  echo $CMD
  eval $CMD
  RET=$?

  if [[ "$RET" != "0" ]]; then
    echo "Cannot create the repo, quitting..."
    return
  fi

  # push the local repo
  git remote remove local
  git remote add local "$USERNAME@$ADDRESS:~/$SUBFOLDER/$SUB_PATH/$REPO_NAME"
  [ -f "$MY_PATH/.gitmodules" ] && git add .gitmodules
  [ -f "$MY_PATH/.gitman.yml" ] && git add .gitman.yml
  git commit -m "switched .gitmodules and .gitman.yml to local"
  git push --all local -u

fi
