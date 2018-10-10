cmake_minimum_required(VERSION 2.8.3)
project(eufs_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  eufs_description
  robot_control
  roscpp
  std_msgs
  std_srvs
  tf
)

catkin_package()

install(PROGRAMS
  nodes/ground_truth_republisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
