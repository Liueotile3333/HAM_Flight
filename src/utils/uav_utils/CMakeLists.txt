cmake_minimum_required(VERSION 2.8.3)
project(uav_utils)

# Header-only 包: 不建库, 仅暴露 include/ 给下游, 并编译一个 gtest 自测。
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_utils
)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 头文件安装(catkin_make install 时暴露给下游)。
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 单元测试。utils.h 经 converters.h 引入 ros/nav_msgs/geometry_msgs,
# 故链 ${catkin_LIBRARIES}; 原代码误链不存在的 ${PROJECT_NAME} 库。
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
