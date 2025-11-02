cmake_minimum_required(VERSION 3.8)
project(autonav_automated_testing)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# RQT Plugin Library
add_library(${PROJECT_NAME}_plugin SHARED
  src/autonav_automated_testing_plugin.cpp
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  include
)

target_link_libraries(${PROJECT_NAME}_plugin
  Qt5::Widgets
  rqt_gui_cpp::rqt_gui_cpp
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  rqt_gui_cpp
  Qt5
)

# Data Publisher Executable
add_executable(data_publisher src/data_publisher.cpp)
target_include_directories(data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(data_publisher
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Export the plugin.xml
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Install targets
install(TARGETS ${PROJECT_NAME}_plugin
  LIBRARY DESTINATION lib
)

install(TARGETS data_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
