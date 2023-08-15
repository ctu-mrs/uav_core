# how to install chrony for syncing time on drones

## README

Chrony **should and will not do automatic step synchronization**, that will break ROS. If you want to do a step sync (the time difference is too large), issue it manually by:
```bash
sudo chronyc -a makestep
```

BTW, the computers should (probably) be in the same timezone.

## install chrony client

```bash
sudo apt-get -y install chrony
sudo cp chrony-client.conf /etc/chrony/chrony.conf
sudo vim /etc/chrony/chrony.conf
```
Edit the lookup server by adding a new line to list of NTP servers.
```
server [SERVER IP ADDRESS - e.g. 192.168.69.5/SERVER HOSTNAME] offline iburst
```
Make sure to have the `SERVER HOSTNAME` in the `/etc/hosts` if you want to use the hostname.

```bash
sudo service chrony restart
```

## how to install chrony server

```bash
sudo apt-get -y install chrony
sudo cp chrony-server.conf /etc/chrony/chrony.conf
sudo vim /etc/chrony/chrony.conf
```
Edit the `allow` ip address in the configuration file that defines the server's accessibility by NTP clients as follows
```
allow [SERVER IP ADDRESS (replace the last digit with zero) - e.g. 192.168.69.0.]/24
```
```bash
sudo service chrony restart
```

## testing chrony

```bash
chronyc tracking
chronyc sources
```
