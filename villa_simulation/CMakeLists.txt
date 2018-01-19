cmake_minimum_required(VERSION 2.8.3)
project(villa_simulation)

find_package(catkin REQUIRED)
catkin_package(
)

install(PROGRAMS
  scripts/stage1_task1_judge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
