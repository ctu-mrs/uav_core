sudo service network-manager stop
sudo ip link set wlan0 down
sudo ip link set ibs0 up
# sudo iw dev wlan0 mode ad-hoc
# sudo iw dev wlan0 channel 8
# sudo iw dev wlan0 ap B6:D6:92:5D:E5:E4
# sudo iw dev wlan0 essid 'test_adhoc'
# sudo iw dev wlan0 key 1234567890
sudo iw dev ibs0 ibss join ibstest 2412 key d:1:5chrs
# sudo ip link set wlan0 up
sudo ip addr add 10.254.239.1/24 broadcast 10.254.239.255 dev ibs0
sudo ip route add 10.254.239.0/24 dev ibs0
# sudo ip addr add 20.20.20.30 dev wlan0
# sudo ifconfig wlan0 netmask 255.255.255.0
# sudo ifconfig wlan0 broadcast 20.20.20.255
