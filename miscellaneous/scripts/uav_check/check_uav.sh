#!/bin/bash

RED='\e[1;91m'
GREEN='\e[32m'
YELLOW='\e[33m'
NC='\e[0m' # No Color
BOLD=$(tput bold)

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DEBUG=0
GOT_PIXHAWK=0

# #{ debug_echo()

function debugecho {
    if [ ! $DEBUG -eq 0 ]
    then
        echo -e "$*"
    fi
}

# #}

# #{ hostname_check()
hostname_check () {
  ret_val=0

  hostname=$( cat /etc/hostname )

  debugecho "/etc/hostname is: $hostname"
  debugecho "UAV_NAME is: $UAV_NAME"
  echo -e "checking if UAV_NAME matches hostname ... \c"
  if [ "$UAV_NAME" = "$hostname" ]
  then
    echo -e "${GREEN}match${NC}"
  else
    echo -e "${RED}not matching${NC}"
    echo -e "${YELLOW}Your /etc/hostname and UAV_NAME should be the same!${NC}"
    ret_val=1
  fi
  return $ret_val
}

# #}

# #{ netplan_check()

netplan_check () {

  ret_val=0

  netplan=$( netplan get )
  network_manager=$( systemctl --type=service | grep 'NetworkManager\|network-manager' )
  wlan0_address="$(netplan get wifis.wlan0.addresses | cut -c3-)"
  wlan0_address=$(echo "$wlan0_address" | tr -d \")
  eth0_address="$(netplan get ethernets.eth0.addresses | cut -c3-)"
  eth0_address=$(echo "$eth0_address" | tr -d \")
  uav_number="${hostname//[!0-9]/}"  #strip all non-numeric chars from hostname, should leave us just with the number of the uav. E.G. -> uav31 -> 31

  if [ "$uav_number" -lt 10 ]
  then
    expected_wlan_ip="192.168.69.10$uav_number/24"
    expected_eth_ip="10.10.20.10$uav_number/24"
  else
    expected_wlan_ip="192.168.69.1$uav_number/24"
    expected_eth_ip="10.10.20.1$uav_number/24"
  fi


  debugecho "Checking network manager:"

# #{ network manager

echo -e "Checking if network manager is running ... \c"
if [ -z "${network_manager}" ]
then
  echo -e "${GREEN}not running${NC}"
else
  echo -e "${RED}running${NC}"
  echo -e "${YELLOW}Run uav_core/miscellaneous/scripts/disable_network_manager.sh${NC}"
  ret_val=1
fi

# #}

debugecho "Checking netplan:"

# #{ wlan0

echo -e "Looking for wlan0 interface ... \c"
if [ -z "$(echo "$netplan" | grep wlan0)" ]
then
  echo -e "${RED}missing${NC}"
  echo -e "${YELLOW}Run uav_core/miscellaneous/scripts/fix_network_interface_names.sh${NC}"
  ret_val=1
else
  echo -e "${GREEN}found${NC}"
fi

echo -e "Looking for wlan0 netplan address ... \c"
if [ -z "$wlan0_address" ]
then
  echo -e "${RED}missing${NC}"
  echo -e "${YELLOW}Set your wlan0 IP address in netplan${NC}"
  ret_val=1
else
  echo -e "${GREEN}found${NC} $wlan0_address"
fi

echo -e "expected wlan0 ip address: $expected_wlan_ip ... \c"
if [ "$expected_wlan_ip" = "$wlan0_address" ]
then
  echo -e "${GREEN}match${NC}"
else
  echo -e "${RED}not matching${NC}"
  echo -e "${YELLOW}Correct your wlan0 ip address!${NC}"
  ret_val=1
fi

# #}

# #{ eth0

echo -e "Looking for eth0 interface ... \c"
if [ -z "$(echo "$netplan" | grep eth0)" ]
then
  echo -e "${RED}missing${NC}"
  echo -e "${YELLOW}Run uav_core/miscellaneous/scripts/fix_network_interface_names.sh${NC}"
  ret_val=1
else
  echo -e "${GREEN}found${NC}"
fi

echo -e "Looking for eth0 netplan address ... \c"
if [ -z "$eth0_address" ]
then
  echo -e "${RED}missing${NC}"
  echo -e "${YELLOW}Set your eth0 IP address in netplan${NC}"
  ret_val=1
else
  echo -e "${GREEN}found${NC} $eth0_address"
fi

debugecho "expected eth0 ip address: $expected_eth_ip ... \c"
if [ "$expected_eth_ip" = "$eth0_address" ]
then
  debugecho "${GREEN}match${NC}"
else
  echo -e "${RED}not matching${NC}"
  echo -e "${YELLOW}Correct your eth0 ip address!${NC}"
  ret_val=1
fi

# #}

return $ret_val

}
# #}

# #{ hosts_check()

hosts_check () {

  ret_val=0

  hosts=$( cat /etc/hosts )
  hostname=$( cat /etc/hostname )
  wlan0_address_noslash=$(echo "$wlan0_address" | cut -f1 -d"/")

  debugecho "Checking /etc/hosts:"

  echo -e "127.0.0.1 localhost ... \c"
  if [ -z "$(echo "$hosts" | grep 127.0.0.1 | grep localhost)" ]
  then
    echo -e "${RED}missing${NC}"
    echo -e "${YELLOW}add this line into /etc/hosts:${NC}"
    echo -e "${YELLOW}127.0.0.1 localhost${NC}"
    ret_val=1
  else
    echo -e "${GREEN}found${NC}"
  fi

  echo -e "127.0.1.1 $hostname ... \c"
  if [ -z "$(echo "$hosts" | grep 127.0.1.1 | grep $hostname)" ]
  then
    echo -e "${RED}missing${NC}"
    echo -e "${YELLOW}add this line into /etc/hosts:${NC}"
    echo -e "${YELLOW}127.0.1.1 $hostname${NC}"
    ret_val=1
  else
    echo -e "${GREEN}found${NC}"
  fi

  echo -e "$wlan0_address_noslash $hostname ... \c"
  if [ -z "$(echo "$hosts" | grep $wlan0_address_noslash | grep $hostname)" ]
  then
    echo -e "${RED}missing${NC}"
    echo -e "${YELLOW}add this line into /etc/hosts:${NC}"
    echo -e "${YELLOW}$wlan0_address_noslash	$hostname${NC}"
    ret_val=1
  else
    echo -e "${GREEN}found${NC}"
  fi

  echo -e "looking for entries with $wlan0_address_noslash ... \c"
  lines_with_address=$(echo "$hosts" | grep -w $wlan0_address_noslash)
  num_lines_with_address=$(echo "$lines_with_address" | wc -l)
  if [ -z "${lines_with_address}" ]
  then
    num_lines_with_address=0
  fi

  if [[ $num_lines_with_address -eq 1 ]]
  then
    echo -e "${GREEN}found 1 entry${NC}, this is correct"
  else
    echo -e "${RED}found $num_lines_with_address ${NC}entries:"
    echo -e "${YELLOW}$lines_with_address${NC}"
    echo -e "${YELLOW}There should only be 1 entry with $wlan0_address_noslash address${NC}"
    ret_val=1
  fi

  return $ret_val

}
# #}

# #{ dev_check()

dev_check () {

  ret_val=0

  debugecho "Checking /dev"
  echo -e "Checking for pixhawk ... \c"
  pixhawk=$(ls /dev | grep pixhawk)

  if [ -z "${pixhawk}" ]
  then
    echo -e "${RED}missing${NC}"
    echo -e "${YELLOW}add pixhawk to udev rules and make sure it is connected!${NC}"
    echo -e "${YELLOW}NOTE: /dev/pixhawk will only be visible if the UAV is powered by a battery!${NC}"
    ret_val=1
  else
    echo -e "${GREEN}found${NC}"
    GOT_PIXHAWK=1
  fi

  if ! [ -z "$(echo "$SENSORS" | grep rplidar)" ]
  then
    rplidar=$(ls /dev | grep rplidar)
    debugecho "Checking for rplidar ... \c"
    if [ -z "${rplidar}" ]
    then
      echo -e "${RED}missing${NC}"
      echo -e "${YELLOW}add rplidar to udev rules and make sure it is connected!${NC}"
      ret_val=1
    else
      debugecho "${GREEN}found${NC}"
    fi
  fi
  return $ret_val
}

# #}

# #{ swap_check()
swap_check () {
  ret_val=0
  debugecho "Checking swap size:"

  num_threads=$(grep -c ^processor /proc/cpuinfo)

  swap=$(grep Swap /proc/meminfo | grep SwapTotal)
  swap="${swap//[!0-9]/}"  #strip all non-numeric chars from hostname, should leave us just with the number of the uav. E.G. -> uav31 -> 31

  ram=$(grep MemTotal /proc/meminfo)
  ram="${ram//[!0-9]/}"  #strip all non-numeric chars from hostname, should leave us just with the number of the uav. E.G. -> uav31 -> 31

  total_mem=$(echo "scale=2; (($ram + $swap) / 1048576)" | bc -l)
  req_mem=$(echo "scale=2; $num_threads * 2.5" | bc -l)

  debugecho "Total swap + RAM size: $total_mem GB"
  debugecho "Recommended size: $req_mem GB ... \c"

  if (( $(echo "$total_mem > $req_mem" |bc -l) ))
  then
    debugecho "${GREEN}pass${NC}"
  else
    debugecho "${RED}fail${NC}"
    echo -e "${RED}Increase you swap size, you can run out of memory when you compile the system${NC}"
    ret_val=1
  fi

  return $ret_val
}

# #}

# #{ broadcast_check()
broadcast_check () {
  ret_val=0
  debugecho "Checking broadcast ip ... \c"

  if [ "$BROADCAST_IP" = "192.168.69.255" ]
  then
    debugecho "${GREEN}correct${NC}"
  else
    debugecho "${RED}incorrect ${BROADCAST_IP} ${NC}"
    echo -e "${YELLOW}Set up BROADCAST_IP="192.168.69.255" variable in .bashrc!${NC}"
    ret_val=1
  fi

  return $ret_val
}

# #}

# #{ workspace_check()
# $1 - workspace which should be checked
# $2 - if you provide 2 workspaces, the script will check that the first workspace is extending the second one

workspace_check () {
  ret_val=0
  curr_dir=$PWD
  workspace=$1

  echo -e "looking for $workspace ... \c"
  if [[ -d "$HOME/$workspace" ]]
  then
    echo -e "${GREEN}found${NC}"
    cd "$HOME/$workspace"

    echo -e "checking $workspace ... \c"
    workspace_valid=$(catkin locate | grep ERROR)

    if [ -z "${workspace_valid}" ]
    then
      echo -e "${GREEN}valid${NC}"
    else
      echo -e "${RED}$workspace is not a valid ROS workspace${NC}"
      echo -e "${YELLOW}run the mrs_uav_system install script!${NC}"
      ret_val=1
    fi

  else
    echo -e "${RED}missing${NC}"
    echo -e "${YELLOW}run the mrs_uav_system install script!${NC}"
    ret_val=1
  fi
  # check workspace extension
  if [[ $# -eq 2 ]]; then
    should_extend=$2
    echo -e "checking $workspace is extending $should_extend ... \c"
    is_extending=$(catkin config | grep Extending | grep $2)
    if [ -z "${is_extending}" ]
    then
      echo -e "${RED}$workspace is not extending $should_extend${NC}"
      echo -e "${YELLOW}set up the workspaces correctly!${NC}"
      ret_val=1
    else
      echo -e "${GREEN}valid${NC}"
    fi
  fi

# check for the march=native flag
echo -e "checking $workspace is not using -march=native ... \c"
march_native=$(catkin config | grep "Additional CMake Args" | grep "march=native")
if [ -z "${march_native}" ]
then
  echo -e "${GREEN}not using${NC}"
else
  echo -e "${RED}found${NC}"
  echo -e "${YELLOW}$workspace has the -march=native flag enabled${NC}"
  echo -e "${YELLOW}This flag is no longer used in the MRS system, remove it${NC}"
  ret_val=1
fi

return $ret_val
}

  # #}

# #{ uav_configurator_check()

uav_configurator_check () {
  ret_val=0

  echo -e "Checking uav_configurator is enabled ... \c"
  configurator_enabled=$(systemctl is-enabled uav_configurator.service)

 if [ "$configurator_enabled" = "enabled" ]
  then
    echo -e "${GREEN}pass${NC}"
  else
    echo -e "${RED}fail${NC}"
    echo -e "${YELLOW}setup the uav_configurator by running the setup script in uav_core/miscellaneous/configurator_scripts${NC}"
    ret_val=1
  fi

  return $ret_val
}

  # #}

# #{ ubuntu20_check()

ubuntu20_check () {
  ret_val=0

  echo -e "Checking ubuntu release ... \c"
  is_20_04=$(lsb_release -id  | grep 20.04)

  if [ -z "${is_20_04}" ]
  then
    echo -e "${RED}fail${NC}"
    echo -e "${YELLOW}You should be running Ubuntu 20.04, you are running:\n$(lsb_release -id)${NC}"
    ret_val=1
  else
    echo -e "${GREEN}pass${NC}"
  fi

  return $ret_val
}

  # #}

# #{ ros_master_check()

ros_master_check () {
  ret_val=0

  echo -e "Checking ROS_MASTER_URI env variable ... \c"
  uri=${ROS_MASTER_URI}

  if [ "$uri" = "http://localhost:11311" ]
  then
    echo -e "${GREEN}correct${NC}"
  else
    echo -e "${RED}incorrect${NC}"
    echo -e "${YELLOW}Your ROS_MASTER_URI should be http://localhost:11311, but it is: ${uri}${NC}\n"
    ret_val=1
  fi

  uri_bashrc=$( cat ~/.bashrc | grep 'ROS_MASTER_URI' | grep -v '#' )

  num_lines_with_uri=$(echo "$uri_bashrc" | wc -l)
  if [ -z "${uri_bashrc}" ]
  then
    num_lines_with_uri=0
  fi

  echo -e "Checking ROS_MASTER_URI in .bashrc ... \c"

  if [[ $num_lines_with_uri -eq 1 ]]
  then
    echo -e "${GREEN}found 1 entry${NC}, this is correct"
    echo -e "Checking value of ROS_MASTER_URI in .bashrc ... \c"
    uri_bashrc_grep=$( echo $uri_bashrc | grep 'export ROS_MASTER_URI=http://localhost:11311' )

    if [ -z "${uri_bashrc_grep}" ]
    then
      echo -e "${RED}fail${NC}"
      echo -e "${YELLOW}There should only be 1 entry in ~/.bashrc: export ROS_MASTER_URI=http://localhost:11311${NC}"
      ret_val=1
    else
      echo -e "${GREEN}correct${NC}"
    fi

  else
    echo -e "${RED}found $num_lines_with_uri ${NC}entries:"
    echo -e "${YELLOW}$uri_bashrc${NC}"
    echo -e "${YELLOW}There should only be 1 entry in ~/.bashrc: export ROS_MASTER_URI=http://localhost:11311${NC}"
    ret_val=1
  fi


  echo -e "Checking ROS_IP env variable ... \c"
  if [ -z "${ROS_IP}" ]
  then
    echo -e "${GREEN}correct${NC}"
  else
    echo -e "${RED}fail${NC}"
    echo -e "${YELLOW}ROS_IP env variable should be empty!${NC}"
    ret_val=1
  fi

  echo -e "Checking ROS_IP in .bashrc variable ... \c"

  ip_bashrc=$( cat ~/.bashrc | grep 'ROS_IP' | grep -v '#' | wc -l )
  if [[ $ip_bashrc -eq 0 ]]
  then
    echo -e "${GREEN}correct${NC}"
  else
    echo -e "${RED}fail${NC}"
    echo -e "${YELLOW}ROS_IP should not be defined in .bashrc!${NC}"
    ret_val=1
  fi

  return $ret_val
}

  # #}
  
  if [[ ( $@ == "--debug") ||  $@ == "-d" ]] 
	then 
		DEBUG=1
	fi
 
  fails=0
  debugecho "\n----------- Ubuntu version check start -----------"
  ubuntu20_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Ubuntu version check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Ubuntu version check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Configurator check start -----------"
  uav_configurator_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Configurator check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Configurator check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Hostname check start -----------"
  hostname_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Hostname check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Hostname check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Netplan check start -----------"
  netplan_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Netplan check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Netplan check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Broadcast check start -----------"
  broadcast_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Broadcast check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Broadcast check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Hosts check start -----------"
  hosts_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Hosts check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Hosts check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Dev check start -----------"
  dev_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Dev check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Dev check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Swap check start -----------"
  swap_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Swap check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Swap check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- Workspace check start -----------"
  workspace_check mrs_workspace
  workspace_check modules_workspace mrs_workspace
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}Workspace check passed${NC} -----------"
  else
    echo -e "----------- ${RED}Workspace check failed${NC} -----------"
    fails=$((fails+1))
  fi

  debugecho "\n----------- ROS_MASTER_URI check start -----------"

  ros_master_check
  if [[ $? -eq 0 ]]
  then
    debugecho "----------- ${GREEN}ROS_MASTER_URI check passed${NC} -----------"
  else
    echo -e "----------- ${RED}ROS_MASTER_URI check failed${NC} -----------"
    fails=$((fails+1))
  fi

  if [[ $fails -eq 0 ]]
  then
    echo -e "----------- ${GREEN}All checks passed${NC} -----------"
  else
    echo -e "\033[1m\n----------- ${RED} ${fails} checks failed${NC} -----------\n\033[0m"
    fails=$((fails+1))
  fi

echo -e "----------- ${BOLD}NOW RUNNING PIXHAWK DIAGNOSTICS${NC} -----------"
sleep 1

rm -f /tmp/pixhawk_config_tmp.txt
if [[ $GOT_PIXHAWK -eq 0 ]]
then
  echo -e "\033[1m\n----------- ${RED}PIXHAWK IS MISSING, SKIPPING PIXHAWK CHECK${NC} -----------\n\033[0m"
else
  bash $SCRIPT_DIR/helper_tmux.sh
fi
cat /tmp/pixhawk_config_tmp.txt
