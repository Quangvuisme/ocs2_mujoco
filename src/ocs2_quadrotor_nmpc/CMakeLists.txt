cmake_minimum_required(VERSION 3.10)
project(ocs2_quadrotor_nmpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

include_directories(include)

# Source files
set(SOURCES
  src/QuadrotorDynamics.cpp
  src/QuadrotorInterface.cpp
  src/QuadrotorUDP.cpp
  src/XboxController.cpp
  src/KeyboardController.cpp
  src/main_udp_nmpc.cpp
)

# Executable
add_executable(ocs2_quadrotor_nmpc_udp ${SOURCES})

ament_target_dependencies(ocs2_quadrotor_nmpc_udp
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_sqp
  ocs2_pinocchio_interface
  ocs2_robotic_tools
)

# Installation
install(TARGETS ocs2_quadrotor_nmpc_udp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

