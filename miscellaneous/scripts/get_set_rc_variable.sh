#!/bin/bash

RCFILE="$HOME/.bashrc"

num=`cat $RCFILE | grep "^export $1" | wc -l`

if [ "$num" -lt "1" ]; then
  
  if [ -z "$3" ]; then
    COMMENTARY=""
  else
    COMMENTARY="# $3"
  fi

  echo "export $1=\"$2\" $COMMENTARY" >> $RCFILE
  echo "$2"
else
  var_name=`eval echo $1`
  var_value=`eval echo -e "\\$${var_name}"`
  echo "$var_value"
fi
