cmake_minimum_required(VERSION 3.16)
project(g1_walking_mujoco LANGUAGES C CXX)

# =============================================================================
# Centroidal NMPC for the Unitree G1 humanoid (12 leg DoF) in MuJoCo - plain CMake, no ROS / ament / colcon.
#
#   cmake -S src/g1_walking_mujoco -B build/g1_walking -DCMAKE_BUILD_TYPE=Release
#   cmake --build build/g1_walking -j
#
# Dependencies: Eigen3, Boost (system filesystem log log_setup), pinocchio 3 (CMake config),
# MuJoCo >= 3.0 (CMake config or MUJOCO_DIR), optional GLFW3 + OpenGL for the GUI / recorder.
# The OCS2 libraries are compiled from ../ocs2_ros2 (see cmake/Ocs2Standalone.cmake).
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib")

option(BUILD_GUI "Build the MuJoCo GUI and the offscreen recorder (needs GLFW + OpenGL)" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---- OCS2 (from source, no ament) ----------------------------------------------
include(Ocs2Standalone)

# ---- MuJoCo -------------------------------------------------------------------
find_package(mujoco CONFIG QUIET)
if(NOT TARGET mujoco::mujoco)
  set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "MuJoCo install / release directory (contains include/ and lib/)")
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_DIR}/include" /usr/local/include /opt/mujoco/include)
  find_library(MUJOCO_LIBRARY NAMES mujoco HINTS "${MUJOCO_DIR}/lib" /usr/local/lib /opt/mujoco/lib)
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "MuJoCo not found. Install it with CMake config files or set -DMUJOCO_DIR=/path/to/mujoco-3.x.y")
  endif()
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  message(STATUS "MuJoCo: ${MUJOCO_LIBRARY}")
endif()

# ---- GUI dependencies -----------------------------------------------------------
if(BUILD_GUI)
  find_package(glfw3 QUIET)
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL QUIET)
  if(NOT glfw3_FOUND OR NOT OpenGL_FOUND)
    message(WARNING "GLFW3 / OpenGL not found: building the headless executable only")
    set(BUILD_GUI OFF)
  endif()
endif()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY)

# ---- core library: OCS2 problem, controller --------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/common/ModelSettings.cpp
  src/gait/Gait.cpp
  src/gait/GaitSchedule.cpp
  src/gait/LegLogic.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/reference_manager/SwitchedModelReferenceManager.cpp
  src/LeggedRobotPreComputation.cpp
  src/G1Interface.cpp
  src/controller/LegPostureIk.cpp
  src/controller/MujocoG1Bridge.cpp
  src/controller/G1Controller.cpp
  src/FSMController.cpp
  src/KeyboardController.cpp
  src/XboxController.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ocs2::ddp ocs2::mpc ocs2::centroidal_model ocs2::pinocchio_interface ocs2::robotic_tools
  mujoco::mujoco Boost::filesystem Threads::Threads)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wno-unused-parameter -Wno-sign-compare -Wno-ignored-attributes)
set_target_properties(${PROJECT_NAME}_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# ---- executable ------------------------------------------------------------------
add_executable(g1_walking_mujoco src/main_g1_mujoco.cpp)
target_link_libraries(g1_walking_mujoco PRIVATE ${PROJECT_NAME}_core)

if(BUILD_GUI)
  add_library(${PROJECT_NAME}_gui STATIC
    src/mujoco_gui/glfw_adapter.cc
    src/mujoco_gui/glfw_dispatch.cc
    src/mujoco_gui/platform_ui_adapter.cc
    src/mujoco_gui/simulate.cc)
  target_include_directories(${PROJECT_NAME}_gui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_gui")
  target_link_libraries(${PROJECT_NAME}_gui PUBLIC mujoco::mujoco glfw OpenGL::GL Eigen3::Eigen Threads::Threads)
  target_link_libraries(g1_walking_mujoco PRIVATE ${PROJECT_NAME}_gui)
  target_compile_definitions(g1_walking_mujoco PRIVATE G1_WALKING_WITH_GUI)
endif()

# ---- tests ------------------------------------------------------------------------
option(BUILD_TESTING "Build the closed-loop tests" ON)
if(BUILD_TESTING)
  enable_testing()
  # deterministic lock-step runs: stand still, then walk forward without falling
  add_test(NAME g1_stand COMMAND g1_walking_mujoco --headless --duration 8 --walk-start 100)
  add_test(NAME g1_walk COMMAND g1_walking_mujoco --headless --vx 0.3 --goal-x 2.0 --duration 20)
  set_tests_properties(g1_stand g1_walk PROPERTIES TIMEOUT 900 LABELS integration)
endif()

message(STATUS "g1_walking_mujoco: GUI=${BUILD_GUI}")
