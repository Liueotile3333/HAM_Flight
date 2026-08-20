cmake_minimum_required(VERSION 3.0.2)
project(realflight_utils)

## Project policy: keep custom C++ targets on the same C++14 baseline.
add_compile_options(-std=c++14 -Wall -Wextra)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geodesy
  geometry_msgs
  mavros_msgs
  message_filters
  nav_msgs
  nodelet
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_msgs geodesy geometry_msgs mavros_msgs message_filters
                 nav_msgs nodelet quadrotor_msgs roscpp sensor_msgs std_msgs tf
  LIBRARIES rcin_remap odom_remap traj_analyse
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(rcin_remap src/rcin_remap.cpp)
add_library(odom_remap src/odom_remap.cpp)
add_library(traj_analyse src/traj_analyse.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rcin_remap ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_remap ${catkin_EXPORTED_TARGETS})
add_dependencies(traj_analyse ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(rcin_remap
  ${catkin_LIBRARIES}
)

target_link_libraries(odom_remap
  ${catkin_LIBRARIES}
)

target_link_libraries(traj_analyse
  ${catkin_LIBRARIES}
)
