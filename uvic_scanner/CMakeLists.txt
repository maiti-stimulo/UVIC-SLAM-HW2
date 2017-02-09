cmake_minimum_required(VERSION 2.8.3)
project(uvic_scanner)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  nav_msgs
  uvic_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(scan_conversion src/scan_conversion.cpp)
target_link_libraries(scan_conversion ${catkin_LIBRARIES})
add_dependencies(scan_conversion uvic_msgs_gencpp)

add_executable(scan_matching src/scan_matching.cpp)
target_link_libraries(scan_matching ${catkin_LIBRARIES})
add_dependencies(scan_matching uvic_msgs_gencpp)
