cmake_minimum_required(VERSION 3.14)
project(anymal_centroidal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Configure package_path.h
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h
)

###########################################
# Find MuJoCo/GLFW first
###########################################
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

###########################################
# MuJoCo GUI library
###########################################
add_library(mujoco_gui_centroidal_lib STATIC
  src/mujoco_gui/simulate.cc
  src/mujoco_gui/platform_ui_adapter.cc
  src/mujoco_gui/glfw_adapter.cc
  src/mujoco_gui/glfw_dispatch.cc
)

target_include_directories(mujoco_gui_centroidal_lib 
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(mujoco_gui_centroidal_lib
  PUBLIC
    mujoco::mujoco
    glfw
    OpenGL::GL
    ${CMAKE_DL_LIBS}
)

set_target_properties(mujoco_gui_centroidal_lib PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
)

###########################################
# Find ROS2 packages
###########################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###########################################
# Find OCS2 and Pinocchio packages
###########################################
find_package(ocs2_core REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(pinocchio REQUIRED)

###########################################
# Find other dependencies
###########################################
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  ocs2_core
  ocs2_pinocchio_interface
  ocs2_centroidal_model
)

###########################################
# Include directories
###########################################
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

###########################################
# Main executable
###########################################
add_executable(anymal_centroidal_gui
  src/main_centroidal_gui.cpp
)

ament_target_dependencies(anymal_centroidal_gui ${dependencies})

target_link_libraries(anymal_centroidal_gui
  mujoco_gui_centroidal_lib
  pinocchio::pinocchio
  ${Boost_LIBRARIES}
  mujoco::mujoco
  glfw
  OpenGL::GL
  pthread
)

###########################################
# Install
###########################################
install(TARGETS 
  mujoco_gui_centroidal_lib
  anymal_centroidal_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include
)

ament_package()
