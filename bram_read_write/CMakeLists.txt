cmake_minimum_required(VERSION 3.5)
project(bram_read_write)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add the directories containing the header files
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/BRAM-uio-driver/src  # Add the directory with bram_uio.h
)

# Add executable
add_executable(bram_read_write_node src/bram_read_write_node.cpp src/BRAM-uio-driver/src/bram_uio.cpp)

# Link dependencies
ament_target_dependencies(bram_read_write_node rclcpp sensor_msgs cv_bridge OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
install(TARGETS
	bram_read_write_node
	DESTINATION lib/${PROJECT_NAME})

ament_package()
