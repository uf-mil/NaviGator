#!/usr/bin/env bash

# remove altered .bashrc and move new one back
rm ~/.bashrc
cp ~/.temp_bashrc ~/.bashrc
# flush ethernet changes
sudo ifconfig eth0 0.0.0.0
# remove altered hosts list and move new one back
sudo mv /etc/temp_hosts /etc/hosts
# back to user land
sudo -k
# source that junk
source ~/.bashrc
