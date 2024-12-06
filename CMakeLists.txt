cmake_minimum_required(VERSION 3.5)
project(hikvision_ros2_driver)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Specify the paths for Hikvision API and camera libraries
find_library(HIKVISION_API_LIBRARY hikvision_api PATHS /usr/local/lib)
find_library(MVCAMERA_LIBRARY MvCameraControl PATHS $ENV{MVCAM_COMMON_RUNENV}/64)

# Include directories for Hikvision API and other dependencies
include_directories(
  /usr/local/include
)

# Declare the executable
add_executable(image_publisher src/image_publisher.cpp)

# Use ament_target_dependencies to link dependencies
ament_target_dependencies(image_publisher
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

# Manually link additional libraries
target_link_libraries(image_publisher
  ${HIKVISION_API_LIBRARY}
  ${MVCAMERA_LIBRARY}
  pthread
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the executable
install(TARGETS image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()