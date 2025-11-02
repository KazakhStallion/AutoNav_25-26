cmake_minimum_required(VERSION 3.8)
project(autonav_automated_testing)

find_package(ament_cmake REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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

# Export the plugin.xml
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

install(TARGETS ${PROJECT_NAME}_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
