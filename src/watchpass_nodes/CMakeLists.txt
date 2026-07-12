cmake_minimum_required(VERSION 3.8)
project(watchpass_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(watchpass_msgs REQUIRED)
find_package(watchpass_framework REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Example image-processing nodes built on FrameProcessorNode. Each is both a
# component (for zero-copy chaining in one container) and a standalone executable.
add_library(watchpass_nodes_components SHARED
  src/grayscale_node.cpp
  src/flip_node.cpp
)
target_include_directories(watchpass_nodes_components PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(watchpass_nodes_components ${OpenCV_LIBRARIES})
ament_target_dependencies(watchpass_nodes_components
  rclcpp
  rclcpp_components
  watchpass_msgs
  watchpass_framework
)

rclcpp_components_register_node(watchpass_nodes_components
  PLUGIN "watchpass_nodes::GrayscaleNode"
  EXECUTABLE grayscale
)
rclcpp_components_register_node(watchpass_nodes_components
  PLUGIN "watchpass_nodes::FlipNode"
  EXECUTABLE flip
)

install(TARGETS watchpass_nodes_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
