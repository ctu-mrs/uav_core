#!/bin/bash
if [ "$UID" -eq 0 ]
    then echo "Please do not run with sudo (but sudo password will be asked when running the script, I just need the name of the current user)"
    exit
fi

username="$USER"

sudo echo "current user: $username"

RED='\e[31m'
GREEN='\e[32m'
YELLOW='\e[33m'
NC='\e[39m' # No Color
BOLD=$(tput bold)
NORMAL=$(tput sgr0)

menu () {
  ret_val=0
  options=("$@")
  PS3="Select a number: "

  select option in "${options[@]}"
  do
    valid_choice=$(echo ${options[@]} | grep -ow "$option" | wc -w)
    if [[ $valid_choice -gt 0 ]]
    then
      echo "Selected: $option"
      return $REPLY
    else
      echo -e "$RED""invalid choice, try again $NC"
    fi


  done

  return $ret_val
}

echo -e "$YELLOW"
echo -e "\n This script will setup your computer to be used on a real UAV. It will:"
echo -e " - configure .bashrc variables, and hostname"
echo -e " - setup your network configuration (switch to netplan, disable network manager, fix interface names and remove other netplans),"
echo -e " - setup the wifi switcher,"
echo -e " - disable hibernation,"
echo -e " - setup udev rules for serial devices (And remove old ones)."
echo -e ""
echo -e "$YELLOW"
echo -e "----- .bashrc variables configuration -----"
echo -e "$NC"

# setting UAV_NAME ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your UAV_NAME? (e.g. uav1, uav2, uav10 ...)$NC"
read -e -p "my UAV_NAME is:$BOLD " -i "uav1" uav_name
echo -e "$NORMAL"
echo -e "Setting UAV_NAME to $uav_name"

sed -i "/export UAV_NAME/c\export UAV_NAME=\"$uav_name\"" /home/$username/.bashrc
echo -e ""

# setting UAV_TYPE ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your UAV_TYPE? (e.g. x500, f450, t650, naki, brus ...)$NC"
read -e -p "my UAV_TYPE is:$BOLD " -i "x500" uav_type
echo -e "$NORMAL"
echo -e "Setting UAV_TYPE to $uav_type"

sed -i "/export UAV_TYPE/c\export UAV_TYPE=\"$uav_type\"" /home/$username/.bashrc
echo -e ""

# setting UAV_MASS ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your UAV_MASS? (e.g. 3.0 - in kilograms, mass of the UAV with battery)$NC"
read -e -p "my UAV_MASS is:$BOLD " -i "3.0" uav_mass
echo -e "$NORMAL"
echo -e "Setting UAV_MASS to $uav_mass"

sed -i "/export UAV_MASS/c\export UAV_MASS=\"$uav_mass\"" /home/$username/.bashrc
echo -e ""

# setting RUN_TYPE ----------------------------------------------
echo -e "Setting RUN_TYPE=\"uav\""
sed -i "/export RUN_TYPE/c\export RUN_TYPE=\"uav\"" /home/$username/.bashrc

# setting PIXGARM ----------------------------------------------
echo -e "Setting PIXGARM=\"true\""
sed -i "/export PIXGARM/c\export PIXGARM=\"true\"" /home/$username/.bashrc
echo -e ""

echo -e "$YELLOW"
echo -e "----- udev rules configuration -----"
echo -e "$NC"

echo -e "$GREEN"
echo -e "Which udev rules should be applied?$NC"

udev_rules_full_path=()
udev_rules=()

search_dir=../udev_rules
for entry in "$search_dir"/*.rules
do
  udev_rules_full_path+=($entry)
  udev_rules+=($(echo "$entry" | cut -c $((${#search_dir}+2))-))
done

menu "${udev_rules[@]}"
selected=$(($?-1))

echo -e ""
echo -e "Removing old udev rules (if they exist)..."
for entry in "${udev_rules[@]}"
do
  echo "removing $entry"
  cmd="sudo rm /etc/udev/rules.d/$entry 2> /dev/null"
  eval "$cmd"
done

echo -e "$GREEN"
echo -e "Applying new udev_rulse:$NC ${udev_rules[$selected]}"
cmd="sudo cp ${udev_rules_full_path[$selected]} /etc/udev/rules.d/"
echo "executing: $cmd"
eval "$cmd"

# setting NETWORK ----------------------------------------------

echo -e "$YELLOW"
echo -e "----- network configuration -----"
echo -e "$NC"

echo -e "$GREEN"
echo -e "Fixing network interface names$NC"
./fix_network_interface_names.sh
echo -e ""

echo -e "$GREEN"
echo -e "Setting up netplan config$NC"

cmd="cp ../network_settings/01-netcfg.yaml /tmp"
echo "executing: $cmd"
eval "$cmd"

possible_uav_wifi_ip_address="192.168.69.100"
possible_uav_eth_ip_address="10.10.20.100"
possible_uav_number=$(echo "$uav_name" | grep -o -E '[0-9]+')
if [[ ${#possible_uav_number} -eq 1 ]]; then
  possible_uav_wifi_ip_address="192.168.69.10$possible_uav_number"
  possible_uav_eth_ip_address="10.10.20.10$possible_uav_number"
fi
if [[ ${#possible_uav_number} -eq 2 ]]; then
  possible_uav_wifi_ip_address="192.168.69.1$possible_uav_number"
  possible_uav_eth_ip_address="10.10.20.1$possible_uav_number"
fi

# # setting WiFi SSID ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your WiFi SSID? (usually mrs_ctu)$NC"
read -e -p "my WiFi SSID is:$BOLD " -i "mrs_ctu" wifi_ssid
echo -e "$NORMAL"
echo -e "Setting WiFi SSID to $wifi_ssid"
sed -i "/mrs_ctu/c\        \"$wifi_ssid\":" /tmp/01-netcfg.yaml

# # setting WiFi password ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your WiFi password? (usually mikrokopter)$NC"
read -e -p "my WiFi password is:$BOLD " -i "mikrokopter" wifi_pass
echo -e "$NORMAL"
echo -e "Setting WiFi password to $wifi_pass"
sed -i "/mikrokopter/c\          password: \"$wifi_pass\"" /tmp/01-netcfg.yaml

# # setting WiFi IP ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your WiFi IP address? (usually 192.168.69.1$BOLD""35$NORMAL for uav$BOLD""35$NORMAL)$NC"
echo -e "Your UAV_NAME is: $BOLD""$uav_name"
read -e -p "my WiFi IP address is:$BOLD " -i "$possible_uav_wifi_ip_address" wifi_ip_address
echo -e "$NORMAL"
echo -e "Setting WiFi IP address to $wifi_ip_address"
sed -i "/addresses: \[192/c\      addresses: \[$wifi_ip_address\/24]" /tmp/01-netcfg.yaml

# # setting WiFi Gateway ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your WiFi Gateway address? (usually 192.168.69.1)$NC"
read -e -p "my WiFi Gateway address is:$BOLD " -i "192.168.69.1" wifi_gateway_address
echo -e "$NORMAL"
echo -e "Setting WiFi Gateway address to $wifi_gateway_address"
sed -i "/gateway4: /c\      gateway4: $wifi_gateway_address" /tmp/01-netcfg.yaml

# # setting Ethernet IP ----------------------------------------------
echo -e "$GREEN"
echo -e "What is your Ethernet IP address? (usually 10.10.20.1$BOLD""35$NORMAL for uav$BOLD""35$NORMAL)$NC"
echo -e "Your UAV_NAME is: $BOLD""$uav_name"
read -e -p "my Ethernet IP address is:$BOLD " -i "$possible_uav_eth_ip_address" eth_ip_address
echo -e "$NORMAL"
echo -e "Setting Ethernet IP address to $eth_ip_address"
sed -i "/addresses: \[10/c\      addresses: \[$eth_ip_address\/24]" /tmp/01-netcfg.yaml

echo -e ""
echo -e "Copying netplan to /etc/netplan and removing any other netplans ..."
sudo rm /etc/netplan/*
sudo cp /tmp/01-netcfg.yaml /etc/netplan/
echo -e ""
echo -e "Setting up WiFi switcher ...."
sudo ../configurator_scripts/setup_configurator_call_with_sudo.sh

echo -e ""
echo -e "Disabling hibernation ...."
sudo  ./disable_hibernation.sh

echo -e "$RED"
echo -e " Network settings will now be applied, NetworkManager will be disabled and Netplan will be applied"
echo -e "$BOLD If you are connected over SSH, you will most likely lose the connection.$NORMAL"
echo -e "$RED If you set up the network configuration correctly, the PC will reboot and connect to your WiFi network with the new IP address$NC"

while true; do
  read -p " Do you wish to proceed (y/n)? " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) exit;;
        * ) echo "Please answer y or n";;
    esac
done

echo -e ""
echo -e "Disabling network manager, applying netplan and rebooting computer ...."
nohup sudo ./disable_network_manager.sh >/dev/null 2>&1; sudo netplan apply >/dev/null 2>&1; sudo reboot now >/dev/null 2>&1
