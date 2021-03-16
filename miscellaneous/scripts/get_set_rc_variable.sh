#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

if [ -z $1 ]; then
  RCFILE="$HOME/.bashrc"
else
  RCFILE="$1"
fi

num=`cat $RCFILE | grep "^export $2" | wc -l`

if [ "$num" -lt "1" ]; then

  if [ -z "$4" ]; then
    COMMENTARY=""
  else
    COMMENTARY="# $4"
  fi

  echo "export $2=\"$3\" $COMMENTARY" >> $RCFILE
  # echo "$3"
else
  var_name=`eval echo $2`
  var_value=`eval echo -e "\\$${var_name}"`
  # echo "$var_value"
fi
