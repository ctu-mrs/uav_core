#!/bin/bash

sudo systemctl stop NetworkManager.service
sudo systemctl disable NetworkManager.service

sudo systemctl stop NetworkManager-wait-online.service
sudo systemctl disable NetworkManager-wait-online.service

sudo systemctl stop NetworkManager-dispatcher.service
sudo systemctl disable NetworkManager-dispatcher.service

sudo systemctl stop network-manager.service
sudo systemctl disable network-manager.service

sudo netplan apply
