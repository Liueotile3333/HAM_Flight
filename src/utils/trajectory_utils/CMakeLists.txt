cmake_minimum_required(VERSION 3.0.2)
project(trajectory_utils)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  quadrotor_msgs
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp nodelet quadrotor_msgs
  DEPENDS EIGEN3
)

include_directories(
   include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/add_traj.cpp
# )
# planning node
add_library(sim_odom
  src/add_traj.cpp
)

target_link_libraries(sim_odom
  ${catkin_LIBRARIES}
)

add_dependencies(sim_odom
  ${catkin_EXPORTED_TARGETS}
)
