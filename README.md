# FOLEX
Four-Legged Experimental Robot

## REQUIREMENTS
* Ubuntu 20.04
* ROS Noetic

## CLONE AND BUILD PACKAGE
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/danichoi737/folex.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd ..
$ catkin_make
```

## ADD USB RULES
```
$ cd ~/catkin_ws/src/folex/folex_controller/scripts
$ chmod +x create_udev_rules
$ roscore
$ rosrun folex_controller create_udev_rules
```
