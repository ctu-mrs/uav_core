# interfaces identification
 Check naming of interfaces by command `ifconfig` because your network interfaces have to be identified as **eth0** and **wlan0**. 
 If they are not, modify the file `/etc/default/grub` where line with **GRUB_CMDLINE_LINUX=""** has to be change to **GRUB_CMDLINE_LINUX="net.ifnames=0 biosdevname=0"**. 
 Finally, call in terminal `sudo update-grub` and `sudo update-grub2` and reboot machine.

# Configuration for static ip addresses

```bash
sudo cp 01-netcfg.yaml /etc/netplan/.
```

Modify the ip address for **eth0** and **wlan0** depanding on the name of the uav. 
For example: 
* **uav2** will have address for **eth0** *10.10.20.202* and **wlan0** *192.168.0.102* 
* **uav6** will have address for **eth0** *10.10.20.206* and **wlan0** *192.168.0.106* 

```bash
sudo vim /etc/netplan/01-netcfg.yaml
```

Remove network manager netplan
```bash
sudo rm /etc/netplan/02-network-manager-all.yaml
```

To apply changes call 
```bash
sudo netplan apply
```

### For older version of ubuntu than 18.04 LTS
```bash
sudo cp interfaces /etc/network/interfaces
```

Modify the ip address for **eth0** and **wlan0** depanding on the name of the uav. 
For example: 
* **uav2** will have address for **eth0** *10.10.20.202* and **wlan0** *192.168.0.102* 
* **uav6** will have address for **eth0** *10.10.20.206* and **wlan0** *192.168.0.106* 

```bash
sudo vim /etc/network/interfaces
```

Finally, reboot machine to apply changes.