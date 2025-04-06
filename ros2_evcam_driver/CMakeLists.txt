cmake_minimum_required(VERSION 3.5)
project(ros2_evcam_driver)

# C++17を利用
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(MetavisionSDK COMPONENTS core driver REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV COMPONENTS highgui REQUIRED)

# evcam_msgs パッケージ（カスタムメッセージ）の依存関係を追加
find_package(evcam_msgs REQUIRED)

# Publisher ノード
add_executable(event_histogram_publisher_node
  src/event_histogram_publisher_node.cpp
  src/Buffer.cpp
  src/EventHistogram.cpp
)
target_include_directories(event_histogram_publisher_node PRIVATE
    ${PROJECT_SOURCE_DIR}/include/ros2_evcam_driver
)
ament_target_dependencies(event_histogram_publisher_node
  rclcpp sensor_msgs cv_bridge
  MetavisionSDK Boost OpenCV
  evcam_msgs
)

# Visualizer ノード
add_executable(event_histogram_visualizer_node
  src/event_histogram_visualizer_node.cpp
  src/Visualization.cpp
)

target_include_directories(event_histogram_visualizer_node PRIVATE
    ${PROJECT_SOURCE_DIR}/include/ros2_evcam_driver
)

ament_target_dependencies(event_histogram_visualizer_node
  rclcpp sensor_msgs cv_bridge
  MetavisionSDK Boost OpenCV
  evcam_msgs
)

install(TARGETS
  event_histogram_publisher_node
  event_histogram_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
