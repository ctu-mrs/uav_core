# copy file and modify it according to UAV's ftdi
# their list is in /dev/serial/by-id/
# 
sudo cp 99-usb-serial.rules /etc/udev/rules.d/.

# Parsing ID's for udev rules
For this use the following command
```
dmesg
```
unplug the device, then plug it in and run the command