cmake_minimum_required(VERSION 3.0.2)
project(estimator)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  quadrotor_msgs
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp nodelet
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
   include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/estimator.cpp
# )
# planning node
add_library(sim_odom
  src/add_traj_bias.cpp
)

target_link_libraries(sim_odom
  ${catkin_LIBRARIES}
)

add_dependencies(sim_odom
  ${catkin_EXPORTED_TARGETS}
)
