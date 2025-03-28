cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)
set(CMAKE_VERBOSE_MAKEFILE "false")
# 修改为以下内容（保留原有 Release 配置，添加 C++17 支持）
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)  # 新增这行
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 新增这行
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")  # 修改这行
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")  # 修改这行

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
