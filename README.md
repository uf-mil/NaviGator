#Building
* clone project into catkin workspace/src
* cd back to catkin workspace
* To build arduino files run catkin_make navigator_shooter_firmware_arduino
* To upload the sketch to the arduino run catkin_make navigator_shooter_firmware_arduino-upload



#Switching USB/Board type
* open firmware/CMakeLists.txt
* To change board, edit text after BOARD within generate_arduino_firmware function
* To change USB port path, edit text after PORT within generate_arduino_firmware function
