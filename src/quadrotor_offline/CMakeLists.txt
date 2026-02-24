cmake_minimum_required(VERSION 3.10)
project(quadrotor_offline)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure package_path.h
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h
)

###########################################
# Find ROS2 packages
###########################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###########################################
# Find OCS2 packages
###########################################
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

###########################################
# Find other dependencies
###########################################
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)

# Find MuJoCo
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

###########################################
# Include directories
###########################################
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

###########################################
# Quadrotor library
###########################################
add_library(${PROJECT_NAME}_lib
  src/QuadrotorInterface.cpp
  src/QuadrotorDynamics.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
)

target_link_libraries(${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
)

###########################################
# Main executable - quadrotor_offline_mujoco
###########################################
add_executable(quadrotor_offline_mujoco
  src/main_offline_mujoco.cpp
)

ament_target_dependencies(quadrotor_offline_mujoco
  rclcpp
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
)

target_link_libraries(quadrotor_offline_mujoco
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  mujoco::mujoco
  glfw
  OpenGL::GL
)

###########################################
# Install
###########################################
install(TARGETS 
  ${PROJECT_NAME}_lib
  quadrotor_offline_mujoco
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY 
  robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

install(DIRECTORY 
  include/
  DESTINATION include
)

ament_package()
