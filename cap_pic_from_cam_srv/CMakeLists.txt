cmake_minimum_required(VERSION 3.0.2)
project(cap_pic_from_cam_srv)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  message_generation
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## 添加srv文件
add_service_files(
  FILES
  CaptureImage.srv
)

## 生成服务
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## 添加库
add_library(${PROJECT_NAME}
  src/CameraCapture.cpp
)

## 添加可执行文件
add_executable(capture_service_node src/cap_pic_from_cam_srv.cpp)

## 链接库和依赖项
target_link_libraries(capture_service_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## 添加依赖关系
add_dependencies(capture_service_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs message_runtime
  DEPENDS system_lib OpenCV
)

## 安装目标
install(TARGETS ${PROJECT_NAME} capture_service_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
