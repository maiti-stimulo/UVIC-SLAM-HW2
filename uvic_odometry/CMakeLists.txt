cmake_minimum_required(VERSION 2.8.3)
project(uvic_odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  roscpp
  tf
  )

find_package(
  Eigen3 REQUIRED
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uvic_odometry
  CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs roscpp tf
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(odometry_tf src/odometry_tf.cpp)
target_link_libraries(odometry_tf ${catkin_LIBRARIES})
