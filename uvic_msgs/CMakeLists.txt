cmake_minimum_required(VERSION 2.8.3)
project(uvic_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  SMToKFI.msg
  KFIToGB.msg
  KFGToKFS.msg
  KFSToSM.msg
  )

add_service_files(
  FILES
  LSToPC.srv
  TKFS.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  uvic_msgs
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

