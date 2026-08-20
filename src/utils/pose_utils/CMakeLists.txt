cmake_minimum_required(VERSION 2.8.3)
project(pose_utils)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
  DEPENDS ARMADILLO
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
    )

add_library(pose_utils
  src/pose_utils.cpp
)

target_link_libraries(pose_utils
  ${ARMADILLO_LIBRARIES}
  ${catkin_LIBRARIES}
)
