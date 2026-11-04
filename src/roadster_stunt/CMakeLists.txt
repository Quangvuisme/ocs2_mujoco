cmake_minimum_required(VERSION 3.10)
project(roadster_stunt)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Configure package_path.h
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h
)

###########################################
# Find OCS2 packages
###########################################
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
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
# Roadster library: OCS2 problem + jet controller
###########################################
add_library(${PROJECT_NAME}_lib
  src/RoadsterDynamics.cpp
  src/ClearanceConstraint.cpp
  src/StuntReference.cpp
  src/RoadsterInterface.cpp
  src/JetController.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
)

target_link_libraries(${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
)

###########################################
# Main executable - roadster_stunt_mujoco
###########################################
add_executable(roadster_stunt_mujoco
  src/main_roadster_mujoco.cpp
)

ament_target_dependencies(roadster_stunt_mujoco
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
)

target_link_libraries(roadster_stunt_mujoco
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
  roadster_stunt_mujoco
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
