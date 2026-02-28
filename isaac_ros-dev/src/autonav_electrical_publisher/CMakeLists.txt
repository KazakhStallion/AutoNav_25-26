cmake_minimum_required(VERSION 3.8)
project(autonav_electrical_publisher)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonav_interfaces REQUIRED)

# Electrical Publisher Executable
add_executable(electrical_publisher src/electrical_publisher.cpp)
target_include_directories(electrical_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${autonav_interfaces_INCLUDE_DIRS}
)
ament_target_dependencies(electrical_publisher
  rclcpp
  std_msgs
  autonav_interfaces
)

# Link i2c library (uncomment if using smbus functions)
# target_link_libraries(electrical_publisher i2c)

# Install targets
install(TARGETS electrical_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
