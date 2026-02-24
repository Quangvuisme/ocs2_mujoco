cmake_minimum_required(VERSION 3.14)
project(quadruped_nmpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

# =============================================================================
# Find Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pinocchio
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# Boost
find_package(Boost REQUIRED COMPONENTS filesystem system)

# =============================================================================
# Generate Package Path Header
# =============================================================================
set(PACKAGE_SHARE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/quadruped_nmpc/package_path.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/include/quadruped_nmpc/package_path.h"
    @ONLY
)

# =============================================================================
# Include Directories
# =============================================================================
include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# =============================================================================
# Source Files - Core Library
# =============================================================================
set(CORE_SOURCES
    # Common
    src/common/ModelSettings.cpp
    
    # Dynamics
    src/dynamics/LeggedRobotDynamicsAD.cpp
    
    # Constraint
    src/constraint/EndEffectorLinearConstraint.cpp
    src/constraint/FrictionConeConstraint.cpp
    src/constraint/ZeroForceConstraint.cpp
    src/constraint/ZeroVelocityConstraintCppAd.cpp
    src/constraint/NormalVelocityConstraintCppAd.cpp
    
    # Foot Planner
    src/foot_planner/SplineCpg.cpp
    src/foot_planner/SwingTrajectoryPlanner.cpp
    src/foot_planner/CubicSpline.cpp
    
    # Gait
    src/gait/LegLogic.cpp
    src/gait/GaitSchedule.cpp
    src/gait/ModeSequenceTemplate.cpp
    src/gait/Gait.cpp
    
    # Initialization
    src/initialization/LeggedRobotInitializer.cpp
    
    # Reference Manager
    src/reference_manager/SwitchedModelReferenceManager.cpp
    
    # Main interface
    src/LeggedRobotInterface.cpp
    src/LeggedRobotPreComputation.cpp
    
    # FSM and Control
    src/FSMController.cpp
    
    # UDP and Xbox
    src/QuadrupedUDP.cpp
    src/XboxController.cpp
    src/KeyboardController.cpp
)

# =============================================================================
# Quadruped NMPC Library
# =============================================================================
add_library(${PROJECT_NAME}_lib SHARED
    ${CORE_SOURCES}
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_directories(${PROJECT_NAME}_lib PUBLIC
    ${pinocchio_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
    ${pinocchio_LIBRARIES}
    ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_lib
    ocs2_core
    ocs2_ddp
    ocs2_mpc
    ocs2_ipm
    ocs2_sqp
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    ocs2_centroidal_model
    ocs2_robotic_assets
)

# =============================================================================
# Main Executable
# =============================================================================
add_executable(${PROJECT_NAME}_main
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_main
    ${PROJECT_NAME}_lib
    pthread
)

ament_target_dependencies(${PROJECT_NAME}_main
    ocs2_core
    ocs2_ddp
    ocs2_mpc
    ocs2_ipm
    ocs2_sqp
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    ocs2_centroidal_model
)

# =============================================================================
# Install
# =============================================================================
install(TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_main
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/quadruped_nmpc/package_path.h
    DESTINATION include/quadruped_nmpc
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY robots/
    DESTINATION share/${PROJECT_NAME}/robots
)

# =============================================================================
# Export and ament_package
# =============================================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ocs2_core
    ocs2_ddp
    ocs2_mpc
    ocs2_ipm
    ocs2_sqp
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    ocs2_centroidal_model
    ocs2_robotic_assets
)

ament_package()
