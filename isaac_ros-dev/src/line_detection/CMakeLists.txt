cmake_minimum_required(VERSION 3.18)  # 3.18+ recommended for CUDA language
project(line_detection LANGUAGES CXX CUDA)

# Warnings for host code
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- ROS & deps ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autonav_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- CUDA Toolchain (modern) ----
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# Jetson Orin Nano is Ampere (SM 87)
# CMake will pass -gencode flags accordingly; no need to hand-write NVCC flags.
set(CMAKE_CUDA_ARCHITECTURES 87)

# ---- Sources ----
file(GLOB SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu
)

add_executable(line_detector ${SRC_FILES})

# C++ standard
target_compile_features(line_detector PUBLIC cxx_std_17)

# Position-independent code (shared objects friendliness)
set_target_properties(line_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- Includes ----
target_include_directories(line_detector
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${image_geometry_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ---- ROS/ament deps ----
ament_target_dependencies(line_detector
  rclcpp
  autonav_interfaces
  cv_bridge
  tf2_ros
  tf2_eigen
  Eigen3
  image_geometry
  tf2_geometry_msgs
)

# ---- Link CUDA & NPP (modern imported targets) ----
set(_cuda_npp_targets CUDA::cudart CUDA::nppist CUDA::nppc)

# Fallback if CUDAToolkit imported targets don’t define nppist on this CMake/Toolkit combo.
foreach(tgt IN LISTS _cuda_npp_targets)
  if(NOT TARGET ${tgt})
    # Fallback to absolute libraries if necessary (rare on JP6, but safe)
    # We prefer the canonical aarch64 targets path on Jetson:
    list(APPEND _cuda_fallback_libs
      /usr/local/cuda/targets/aarch64-linux/lib/libcudart.so
      /usr/local/cuda/targets/aarch64-linux/lib/libnppist.so
      /usr/local/cuda/targets/aarch64-linux/lib/libnppc.so
    )
    set(_use_fallback_libs TRUE)
    break()
  endif()
endforeach()

if(_use_fallback_libs)
  target_link_libraries(line_detector ${_cuda_fallback_libs})
  # Bake rpath so the binary finds CUDA libs at runtime
  target_link_options(line_detector PRIVATE -Wl,-rpath,/usr/local/cuda/targets/aarch64-linux/lib)
else()
  target_link_libraries(line_detector ${_cuda_npp_targets})
  # Many JetPack images already have loader paths set, but bake rpath for robustness
  target_link_options(line_detector PRIVATE -Wl,-rpath,$<TARGET_FILE_DIR:CUDA::cudart>)
endif()

# ---- Install ----
install(TARGETS line_detector DESTINATION lib/${PROJECT_NAME})

# ---- Tests / Linters ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
