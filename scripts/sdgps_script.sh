#!/bin/sh

#Launches GPS into existing sdgps screen
sudo -u navigator    screen   -S sdgps -p 0 -X stuff ' cd ~/sdgps/sdgps_current\n'
sudo -u navigator -i screen   -S sdgps -p 0 -X stuff ' sudo build/main sylphase-usbgpsimu2 --antenna-position "[0.37465,-0.136525,0.587502]" --output-samples ! listen-cooked-tcp 1236 ! tracker ! listen-observables-tcp 1235\n'
#sudo -u navigator    screen   -S sdgps -p 0 -X stuff ' sudo build/main read-observables-file /home/navigator/bags/sdgps/standalone_1477228836.obs --rate 10 ! listen-observables-tcp 1235\n'
sudo -u navigator    screen   -S sdgps -X screen -t 1
sudo -u navigator    screen   -S sdgps -p 1 -X stuff ' cd ~/sdgps/sdgps_current\n'
sudo -u navigator    screen   -S sdgps -p 1 -X stuff ' build/main connect-cooked-tcp 127.0.0.1 1236 ! cooked-to-raw ! write-raw-file /home/navigator/bags/sdgps/last.raw\n'
sudo -u navigator    screen   -S sdgps -X screen -t 2
sudo -u navigator    screen   -S sdgps -p 2 -X stuff ' cd ~/sdgps/sdgps_current\n'
sudo -u navigator    screen   -S sdgps -p 2 -X stuff ' build/main connect-observables-tcp 127.0.0.1 1235 ! write-observables-file /home/navigator/bags/sdgps/kf2_`date +%s`.obs ! kf2 --decimation 10 ! listen-solution-tcp 1234\n'
sudo -u navigator    screen   -S sdgps -X screen -t 3
sudo -u navigator    screen   -S sdgps -p 3 -X stuff ' cd ~/sdgps/sdgps_current\n'
sudo -u navigator    screen   -S sdgps -p 3 -X stuff ' scripts/solution_ros_bridge _port:=1234 _child_frame_id:=/ins /odom:=/ins_odom _force_z_to_zero:=true\n'
