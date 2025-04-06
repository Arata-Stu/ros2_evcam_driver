find_package(rclcpp REQUIRED)
find_package(evcam_msgs REQUIRED)

add_executable(histogram_accumulator_node src/histogram_accumulator_node.cpp)
ament_target_dependencies(histogram_accumulator_node rclcpp evcam_msgs)

install(TARGETS
  histogram_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)
