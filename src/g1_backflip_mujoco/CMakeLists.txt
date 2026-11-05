cmake_minimum_required(VERSION 3.16)
project(g1_backflip_mujoco LANGUAGES C CXX)

# =============================================================================
# Standing backflip of the 29-DoF Unitree G1 in MuJoCo with the whole-body NMPC of wb_humanoid_mpc
# (https://github.com/manumerous/wb_humanoid_mpc, git submodule src/wb_humanoid_mpc). Plain CMake, no ROS.
#
#   git submodule update --init src/wb_humanoid_mpc
#   git -C src/wb_humanoid_mpc submodule update --init lib/ocs2_ros2
#   cmake -S src/g1_backflip_mujoco -B build/g1_backflip -DCMAKE_BUILD_TYPE=Release
#   cmake --build build/g1_backflip -j
#
# Dependencies: Eigen3, Boost (filesystem log log_setup), pinocchio 3, urdfdom, MuJoCo >= 3.0 (CMake config or
# MUJOCO_DIR), git + network for BLASFEO / HPIPM (FetchContent), optional GLFW3 + OpenGL for the GUI / recorder.
# =============================================================================

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib")

option(BUILD_GUI "Build the MuJoCo GUI and the offscreen recorder (needs GLFW + OpenGL)" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---- wb_humanoid_mpc + its OCS2 fork (from source, no ament) ------------------------
include(WbHumanoidMpc)

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

# ---- core library: OCS2 problem, guide, plan, controller ---------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/G1WbBackflipInterface.cpp
  src/constraint/JointTorqueLimitConstraint.cpp
  src/BackflipGuide.cpp
  src/BackflipPlanner.cpp
  src/controller/MujocoG1Bridge.cpp
  src/controller/BackflipController.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(${PROJECT_NAME}_core PUBLIC wbmpc::humanoid_wb_mpc mujoco::mujoco Boost::filesystem Threads::Threads)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wno-unused-parameter -Wno-sign-compare -Wno-ignored-attributes)
set_target_properties(${PROJECT_NAME}_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# ---- executables -------------------------------------------------------------------
add_executable(g1_backflip_plan src/main_plan.cpp)
target_link_libraries(g1_backflip_plan PRIVATE ${PROJECT_NAME}_core)

add_executable(g1_backflip_mujoco src/main_g1_backflip.cpp)
target_link_libraries(g1_backflip_mujoco PRIVATE ${PROJECT_NAME}_core)

if(BUILD_GUI)
  add_library(${PROJECT_NAME}_gui STATIC
    src/mujoco_gui/glfw_adapter.cc
    src/mujoco_gui/glfw_dispatch.cc
    src/mujoco_gui/platform_ui_adapter.cc
    src/mujoco_gui/simulate.cc)
  target_include_directories(${PROJECT_NAME}_gui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_gui")
  target_link_libraries(${PROJECT_NAME}_gui PUBLIC mujoco::mujoco glfw OpenGL::GL Eigen3::Eigen Threads::Threads)
  target_link_libraries(g1_backflip_mujoco PRIVATE ${PROJECT_NAME}_gui)
  target_compile_definitions(g1_backflip_mujoco PRIVATE G1_BACKFLIP_WITH_GUI)
endif()

# ---- tests ------------------------------------------------------------------------
option(BUILD_TESTING "Build the closed-loop test" ON)
if(BUILD_TESTING)
  enable_testing()
  # deterministic lock-step run: plan, backflip, land and stand (exit code 0)
  add_test(NAME g1_backflip COMMAND g1_backflip_mujoco --headless)
  set_tests_properties(g1_backflip PROPERTIES TIMEOUT 3600 LABELS integration)
endif()

message(STATUS "g1_backflip_mujoco: GUI=${BUILD_GUI}")
