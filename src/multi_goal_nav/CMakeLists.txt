cmake_minimum_required(VERSION 3.0.2)
project(multi_goal_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/multi_goal_nav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
