# how to install chrony for syncing time on drones

## README

Chrony **should and will not do automatic step synchronization**, that will break ROS. If you want to do a step sync (the time difference is too large), issue it manually by:
```bash
sudo chronyc -a makestep
```

BTW, the computers should (probably) be in the same timezone.

## install chrony
sudo apt-get -y install chrony
sudo cp chrony-client.conf /etc/chrony/chrony.conf 
sudo service chrony restart

## how to install chrony server
sudo apt-get -y install chrony
sudo cp chrony-server.conf /etc/chrony/chrony.conf
sudo service chrony restart

## testing chrony
chronyc tracking
chronyc sources
