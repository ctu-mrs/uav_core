#!/bin/bash
sed -i 's/mrs_ctu.*"/mrs_ctu_6"/g' /etc/netplan/01-netcfg.yaml
netplan apply

