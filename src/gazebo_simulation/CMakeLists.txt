cmake_minimum_required(VERSION 2.8.3)
project(gazebo_simulation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  control_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
  /usr/lib/python2.7/dist-packages  # Path to installed packages
  ${catkin_INCLUDE_DIRS}
)

## Mark executables and/or libraries for installation
catkin_install_python(
  PROGRAMS
    src/ui_interface.py
    src/plotResults.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
