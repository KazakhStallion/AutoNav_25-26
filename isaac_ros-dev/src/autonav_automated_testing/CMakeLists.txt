cmake_minimum_required(VERSION 3.8)
project(autonav_automated_testing)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonav_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# RQT Plugin Library
add_library(autonav_automated_testing_plugin SHARED
  src/autonav_automated_testing_plugin.cpp
  include/autonav_automated_testing/autonav_automated_testing_plugin.hpp
)

target_include_directories(autonav_automated_testing_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(autonav_automated_testing_plugin
  Qt5::Widgets
)

ament_target_dependencies(autonav_automated_testing_plugin
  rclcpp
  std_msgs
  rqt_gui_cpp
  pluginlib
  ament_index_cpp
)

# Data Publisher Executable
add_executable(data_publisher src/data_publisher.cpp)
target_include_directories(data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${autonav_interfaces_INCLUDE_DIRS}
)
ament_target_dependencies(data_publisher
  rclcpp
  std_msgs
  autonav_interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Export the plugin.xml
pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

# Install targets
install(TARGETS autonav_automated_testing_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS data_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config images src
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
