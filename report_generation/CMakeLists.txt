cmake_minimum_required(VERSION 2.8.3)
project(report_generation)

find_package(catkin REQUIRED
COMPONENTS
std_msgs
rospy
message_generation
sensor_msgs
)

 add_service_files(
   FILES
   RecordCupboard.srv
   RecordFinalCupboard.srv
 )

catkin_python_setup()

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS
message_runtime
)

