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
find_package(yaml-cpp    REQUIRED)
find_package(OpenCV      REQUIRED COMPONENTS core imgcodecs imgproc)

# =============================================================================
# Нода 1: stereo_camera_node
# =============================================================================
add_executable(stereo_camera_node
  src/stereo_camera_node.cpp
  src/mjpg_capture.cpp
  src/frame_grabber.cpp
)

target_include_directories(stereo_camera_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(stereo_camera_node
  rclcpp sensor_msgs std_msgs cv_bridge
)

target_link_libraries(stereo_camera_node
  ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES}
)

# =============================================================================
# Нода 2: image_decompressor_node
# =============================================================================
add_executable(image_decompressor_node
  src/image_decompressor_node.cpp
  # Добавьте сюда общие файлы, если они нужны этой ноде
  # src/mjpg_capture.cpp
  # src/frame_grabber.cpp
)

target_include_directories(image_decompressor_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(image_decompressor_node
  rclcpp sensor_msgs std_msgs cv_bridge
)

target_link_libraries(image_decompressor_node
  ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES}
)

# =============================================================================
# Нода 3: stereo_camera_all_cams_qos_compressed (если у неё тоже есть main())
# =============================================================================
add_executable(stereo_camera_all_cams_qos_compressed
  src/stereo_camera_all_cams_qos_compressed.cpp
  src/mjpg_capture.cpp
  src/frame_grabber.cpp
)

target_include_directories(stereo_camera_all_cams_qos_compressed PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(stereo_camera_all_cams_qos_compressed
  rclcpp sensor_msgs std_msgs cv_bridge
)

target_link_libraries(stereo_camera_all_cams_qos_compressed
  ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES}
)

# =============================================================================
# Установка
# =============================================================================
install(TARGETS
  stereo_camera_node
  image_decompressor_node
  stereo_camera_all_cams_qos_compressed
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
