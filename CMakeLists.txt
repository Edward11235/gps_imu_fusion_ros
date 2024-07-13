cmake_minimum_required(VERSION 3.0.2)
project(gps_imu_fusion_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps_imu_fusion_ros
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_compile_options(-std=c++17)  # Add this line to specify C++17

add_library(ekfNavINS src/ekfNavINS.cpp)
target_link_libraries(ekfNavINS ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(imu_gps_fusion src/imu_gps_fusion.cpp)
target_link_libraries(imu_gps_fusion ${catkin_LIBRARIES} ekfNavINS)
