cmake_minimum_required(VERSION 3.14)
project(anymal_walking_online)

# Policies for Boost compatibility
cmake_policy(SET CMP0167 OLD)
cmake_policy(SET CMP0072 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

pkg_check_modules(pinocchio REQUIRED pinocchio)

# Package path header
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/anymal_walking_online/package_path.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/anymal_walking_online/package_path.h"
  @ONLY
)

# Core library
add_library(${PROJECT_NAME}_core SHARED
  # Common
  src/common/ModelSettings.cpp
  # Interface
  src/LeggedRobotInterface.cpp
  src/LeggedRobotPreComputation.cpp
  # Dynamics
  src/dynamics/LeggedRobotDynamicsAD.cpp
  # Constraint
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  # Foot planner
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  # Gait
  src/gait/Gait.cpp
  src/gait/GaitSchedule.cpp
  src/gait/LegLogic.cpp
  src/gait/ModeSequenceTemplate.cpp
  # Initialization
  src/initialization/LeggedRobotInitializer.cpp
  # Reference manager
  src/reference_manager/SwitchedModelReferenceManager.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pinocchio_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME}_core PUBLIC
  ${pinocchio_LIBRARY_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_sqp
  ocs2_ipm
  ocs2_centroidal_model
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_robotic_assets
)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  Boost::filesystem
  ${pinocchio_LIBRARIES}
)

# MuJoCo GUI library
add_library(mujoco_gui STATIC
  src/mujoco_gui/glfw_adapter.cc
  src/mujoco_gui/glfw_dispatch.cc
  src/mujoco_gui/platform_ui_adapter.cc
  src/mujoco_gui/simulate.cc
)

target_include_directories(mujoco_gui PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_gui
)

target_link_libraries(mujoco_gui
  mujoco::mujoco
  glfw
  ${OPENGL_LIBRARIES}
)

# Xbox Controller and FSM library
add_library(${PROJECT_NAME}_controllers SHARED
  src/XboxController.cpp
  src/KeyboardController.cpp
  src/FSMController.cpp
)

target_include_directories(${PROJECT_NAME}_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_controllers
  pthread
)

# Test interface executable
add_executable(test_anymal_interface src/testAnymalInterface.cpp)
target_link_libraries(test_anymal_interface
  ${PROJECT_NAME}_core
)

# MuJoCo simulation executable
add_executable(anymal_walking_mujoco src/main_anymal_mujoco.cpp)
target_include_directories(anymal_walking_mujoco PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_gui
)
target_link_libraries(anymal_walking_mujoco
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_controllers
  mujoco_gui
  mujoco::mujoco
)

# Install
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_controllers mujoco_gui test_anymal_interface anymal_walking_mujoco
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
)

install(DIRECTORY config robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
