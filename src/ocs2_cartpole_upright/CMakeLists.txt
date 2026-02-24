cmake_minimum_required(VERSION 3.14)
project(ocs2_cartpole_upright)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

include_directories(
  include
)

add_executable(cartpole_upright_nmpc
  src/main_udp_nmpc.cpp
  src/CartpoleUDP.cpp
  src/CartPoleInterface.cpp
  src/CartpoleDynamicsPinocchio.cpp
)

ament_target_dependencies(cartpole_upright_nmpc
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
)

install(TARGETS cartpole_upright_nmpc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
