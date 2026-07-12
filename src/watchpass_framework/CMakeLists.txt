cmake_minimum_required(VERSION 3.8)
project(watchpass_framework)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(watchpass_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(watchpass_framework SHARED
  src/frame_view.cpp
  src/frame_consumer_node.cpp
  src/frame_processor_node.cpp
)
target_include_directories(watchpass_framework PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(watchpass_framework ${OpenCV_LIBRARIES})
ament_target_dependencies(watchpass_framework rclcpp builtin_interfaces watchpass_msgs)

install(TARGETS watchpass_framework
  EXPORT export_watchpass_framework
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# Export so downstream packages can just find_package(watchpass_framework).
ament_export_targets(export_watchpass_framework HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp builtin_interfaces watchpass_msgs OpenCV)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
