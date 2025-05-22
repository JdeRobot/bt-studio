cmake_minimum_required(VERSION 3.8)
project(test_world)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===========================================================================
# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  sensor_msgs
  rclcpp
  rclpy
  urdf
  xacro
  ament_cmake_ros
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# ============================================================================
# Resources

install(DIRECTORY

  launch
  models
  worlds

  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
