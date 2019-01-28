#!/bin/bash

actual_dir=`dirname "$0"`
source $actual_dir/script_functions.sh
source $actual_dir/config.sh

ip_setting_wifi=`ifconfig wlan0 2> /dev/null | grep "inet addr:" | awk '{print $1  $2}'`
ip_setting_eth=`ifconfig eth0 2> /dev/null | grep "inet addr:" | awk '{print $1  $2}'`

echo -e "$header_begin""wifi: ""$green_color""$ip_setting_wifi""$header_end"
echo -e "$header_begin""eth: ""$green_color""$ip_setting_eth""$header_end"