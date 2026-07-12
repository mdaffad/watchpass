cmake_minimum_required(VERSION 3.8)
project(watchpass_streamer)

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
find_package(sensor_msgs REQUIRED)
find_package(watchpass_msgs REQUIRED)
find_package(watchpass_framework REQUIRED)

# Shared component library holding both nodes plus the ffmpeg pipe helper.
add_library(watchpass_streamer_components SHARED
  src/camera_bridge_node.cpp
  src/ffmpeg_streamer_node.cpp
  src/ffmpeg_pipe.cpp
  src/ros_image_source.cpp
  src/v4l2_source.cpp
)
target_include_directories(watchpass_streamer_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(watchpass_streamer_components
  rclcpp
  rclcpp_components
  sensor_msgs
  watchpass_msgs
  watchpass_framework
)

# Register each node as a component AND generate a standalone executable.
# Running them in one component_container gives intra-process zero-copy;
# running the executables in separate processes gives shared-memory zero-copy.
rclcpp_components_register_node(watchpass_streamer_components
  PLUGIN "watchpass_streamer::CameraBridgeNode"
  EXECUTABLE camera_bridge
)
rclcpp_components_register_node(watchpass_streamer_components
  PLUGIN "watchpass_streamer::FfmpegStreamerNode"
  EXECUTABLE ffmpeg_streamer
)

install(TARGETS watchpass_streamer_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS scripts/stream_ffmpeg.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
