sudo service network-manager stop
sudo ip link set wlan0 down
sudo iwconfig wlan0 mode ad-hoc
sudo iwconfig wlan0 channel 4
sudo iwconfig wlan0 essid 'name'
sudo iwconfig wlan0 key 1234567890
sudo ip link set wlan0 up
sudo ip addr add 20.20.20.30 dev wlan0
