#Building
* clone project into catkin workspace/src
* cd back to catkin workspace
* To build arduino files run `catkin_make navigator_shooter_firmware_arduino`
* To upload the sketch to the arduino run `catkin_make navigator_shooter_firmware_arduino-upload`

#Running
* Start a ros master node `roscore`
* Start a node to communicate with arduino and turn it into ros stuff `rosrun rosserial_python serial_node.py /dev/USBPATH` ex: `rosrun rosserial_python serial_node.py /dev/ttyACM0`
* Use `rostopic list` to see if the subscribers and publishers you created show up

#Testing
* To see incoming data use `rostopic echo TOPIC` where TOPIC is the topic you are publishing to ex: `rostopic echo chatter`
* To send data to the arduino use `rostopic pub /TOPIC MSGTYPE MSGDATA -1` ex: `rostopic pub /shooter_control std_msgs/Int8 9 -1`


#Switching USB/Board type
* open firmware/CMakeLists.txt
* To change board, edit text after BOARD within generate_arduino_firmware function
* To change USB port path, edit text after PORT within generate_arduino_firmware function
ex:
```
generate_arduino_firmware(arduino
  SRCS shooter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
)
```


