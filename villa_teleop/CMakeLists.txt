cmake_minimum_required(VERSION 2.8.3)
project(villa_teleop)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    std_srvs
    actionlib
    actionlib_msgs
    message_generation
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    roscpp
    rospy
    std_msgs
    std_srvs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

catkin_install_python(PROGRAMS scripts/keyboard_teleop
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
