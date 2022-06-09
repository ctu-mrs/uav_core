#!/bin/bash
sed -i 's/mrs_ctu.*"/mrs_ctu"/g' /etc/netplan/01-netcfg.yaml
netplan apply

