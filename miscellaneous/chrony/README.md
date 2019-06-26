# how to install chrony for syncing time on drones

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
