# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(seniorproject)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy
  sensor_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  nodes/test_lidar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)