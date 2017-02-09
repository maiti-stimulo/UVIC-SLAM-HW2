cmake_minimum_required(VERSION 2.8.3)
project(uvic_graph)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  uvic_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(kf_graph src/kf_graph.cpp)
target_link_libraries(kf_graph ${catkin_LIBRARIES})
add_dependencies(kf_graph uvic_msgs_gencpp)

add_executable(kf_select src/kf_select.cpp)
target_link_libraries(kf_select ${catkin_LIBRARIES})
add_dependencies(kf_select uvic_msgs_gencpp)

add_executable(kf_issuer src/kf_issuer.cpp)
target_link_libraries(kf_issuer ${catkin_LIBRARIES})
add_dependencies(kf_issuer uvic_msgs_gencpp)
