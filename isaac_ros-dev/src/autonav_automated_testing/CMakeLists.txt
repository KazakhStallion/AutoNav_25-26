cmake_minimum_required(VERSION 3.8)
project(autonav_automated_testing)

# Enable warnings for C++ code (if any)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rqt_gui REQUIRED)
find_package(rqt_gui_py REQUIRED)
find_package(python_qt_binding REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# --- Install Python package (so plugin is discoverable) ---
ament_python_install_package(${PROJECT_NAME})

# --- Install Python scripts if you have CLI or launch helpers ---
install(PROGRAMS
  launch/automated_testing.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files directory ---
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# --- Install plugin description XML (for RQt to load it) ---
install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- Optional: resource marker for ament index ---
install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/
)

# --- Testing (optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Declare the package for ament ---
ament_package()
