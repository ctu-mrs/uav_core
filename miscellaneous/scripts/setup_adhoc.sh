sudo service network-manager stop
sudo ip link set wlan0 down
sudo iwconfig wlan0 mode ad-hoc
sudo iwconfig wlan0 channel 4
sudo iwconfig wlan0 ap B6:D6:92:5D:E5:E4
sudo iwconfig wlan0 essid 'test'
sudo iwconfig wlan0 key 1234567890
sudo ip link set wlan0 up
sudo ip addr add 20.20.20.30 dev wlan0
sudo ifconfig wlan0 netmask 255.255.255.0 dev wlan0
sudo ifconfig wlan0 broadcast 20.20.20.255 dev wlan0
