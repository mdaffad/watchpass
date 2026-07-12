cmake_minimum_required(VERSION 3.8)
project(watchpass_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# --- Configurable frame capacity -------------------------------------
# The WatchFrame payload size is a single build-time knob instead of a
# hardcoded literal. Override it per workspace, e.g.:
#   colcon build --cmake-args -DWATCHPASS_FRAME_CAPACITY=6220800   # 1920x1080x3
set(WATCHPASS_FRAME_CAPACITY "2764800" CACHE STRING
  "Maximum WatchFrame payload in bytes (default 1280x720x3 = 2764800)")
message(STATUS "watchpass_msgs: WatchFrame capacity = ${WATCHPASS_FRAME_CAPACITY} bytes")

# Generate WatchFrame.msg from the template into the build tree so the source
# stays clean, then hand rosidl the generated absolute path.
set(_watchframe_base "${CMAKE_CURRENT_BINARY_DIR}/gen")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/msg/WatchFrame.msg.in"
  "${_watchframe_base}/msg/WatchFrame.msg"
  @ONLY)

# For the generated file, pass "<absolute base>:<relative path>" so rosidl keeps
# the intended msg/WatchFrame name; the plain relative form is used otherwise.
rosidl_generate_interfaces(${PROJECT_NAME}
  "${_watchframe_base}:msg/WatchFrame.msg"
  "msg/WatchFrameHandle.msg"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
