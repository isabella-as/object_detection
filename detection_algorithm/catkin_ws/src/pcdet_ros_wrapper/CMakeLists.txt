cmake_minimum_required(VERSION 3.0.2)
project(pcdet_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pcdet_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
