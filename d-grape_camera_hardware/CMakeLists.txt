cmake_minimum_required(VERSION 3.8)
project(d-grape_camera_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(cv_bridge   REQUIRED)
find_package(yaml-cpp   REQUIRED)
find_package(OpenCV      REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(stereo_camera_node
  src/mjpg_capture.cpp
  src/frame_grabber.cpp
  src/stereo_camera_node.cpp
)

target_include_directories(stereo_camera_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(stereo_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(stereo_camera_node
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS stereo_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
