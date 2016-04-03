#!/usr/bin/env bash

# Check for the proper argument - IP Address only for now
if [ "$#" -ne 1 ]; then
    echo "Please enter your NaviGator IP address as the first and only argument"
    echo "FORMAT: 'bash net_setup.sh 192.168.X.XXX'"
    echo "The assigned addresses can be found in the IPs.txt root directory of the NaviGator repository"
    exit
fi

# Copy the .bashrc for replacment later
cp ~/.bashrc ~/.temp_bashrc
# Append the NaviGator computers ROS information
echo "export ROS_MASTER_URI=http://wamv:11311" >> ~/.bashrc

# Configure your ethernet port to the desired port
sudo ifconfig eth0 $1

# Copy the hosts file for replacement later
sudo cp /etc/hosts /etc/temp_hosts

# Add all the NaviGator host info to a temporary file in the current directory
echo "127.0.0.1       localhost" > ./hosts
echo "127.0.1.1       $HOSTNAME" >> ./hosts
echo "192.168.1.101   wamv" >> ./hosts
echo "192.168.1.2     wamv_onboard_router" >> ./hosts
echo "192.168.1.20    wamv_onshore_router" >> ./hosts
echo "192.168.1.201   wamv_lidar" >> ./hosts
echo "192.168.1.111   Marth" >> ./hosts
echo "192.168.1.112   zach" >> ./hosts
echo "::1     ip6-localhost ip6-loopback" >> ./hosts
echo "fe00::0 ip6-localnet" >> ./hosts
echo "ff00::0 ip6-mcastprefix" >> ./hosts
echo "ff02::1 ip6-allnodes" >> ./hosts
echo "ff02::2 ip6-allrouters" >> ./hosts

# Copy the temporary file to the /etc/hosts file
sudo mv ./hosts /etc/hosts
# Get back into user land
sudo -k
# source that junk
source ~/.bashrc


