cmake_minimum_required(VERSION 3.5)
project(event_histogram_accumulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(evcam_msgs REQUIRED)
# C++17を利用
set(CMAKE_CXX_STANDARD 17)
# 追加: includeディレクトリの指定
include_directories(include)

# 追加: HistogramBufferのビルド対象に含める
add_library(histogram_buffer
  src/histogram_buffer.cpp
)
ament_target_dependencies(histogram_buffer rclcpp evcam_msgs)

# 実行ファイルにリンク
add_executable(histogram_accumulator_node src/histogram_accumulator_node.cpp)
ament_target_dependencies(histogram_accumulator_node rclcpp evcam_msgs)
target_link_libraries(histogram_accumulator_node histogram_buffer)

install(TARGETS
  histogram_accumulator_node
  histogram_buffer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
