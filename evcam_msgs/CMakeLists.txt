cmake_minimum_required(VERSION 3.5)
project(evcam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# メッセージファイルの指定
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EventHistogram.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
