#!/bin/bash

if [ "$(id -u)" != "0" ]; then
  exec sudo "$0" "$@" 
fi

echo "Shutting down the computor in 5 s"

sleep 5

sudo shutdown -P now
