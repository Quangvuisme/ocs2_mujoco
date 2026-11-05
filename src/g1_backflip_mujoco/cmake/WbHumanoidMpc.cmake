# =============================================================================
# WbHumanoidMpc.cmake
#
# Builds the whole-body humanoid NMPC of https://github.com/manumerous/wb_humanoid_mpc (git submodule
# src/wb_humanoid_mpc) and the OCS2 fork it is written against (src/wb_humanoid_mpc/lib/ocs2_ros2) with plain CMake:
# no ament, no colcon, no ROS. The upstream CMakeLists.txt files are ament packages, so they are not used; every
# library is compiled from the source list of its upstream add_library() call (all of src/ minus lint targets).
# The ROS-only parts of humanoid_common_mpc (gait command updater, procedural motion manager) and the
# motorium / ROS MRT controller of humanoid_wb_mpc are left out.
#
# BLASFEO and HPIPM (QP solver of the SQP) are downloaded with FetchContent at the commits upstream pins.
#
# Targets (all SHARED unless noted):
#   wbmpc::ocs2_core wbmpc::ocs2_oc wbmpc::ocs2_mpc wbmpc::ocs2_qp_solver wbmpc::ocs2_ddp wbmpc::ocs2_sqp
#   wbmpc::ocs2_robotic_tools wbmpc::ocs2_pinocchio_interface wbmpc::hpipm_interface
#   wbmpc::humanoid_common_mpc wbmpc::humanoid_wb_mpc
#
# Inputs:
#   WB_HUMANOID_MPC_ROOT  path to the wb_humanoid_mpc checkout (default: ../wb_humanoid_mpc)
# =============================================================================

if(TARGET wbmpc_humanoid_wb_mpc)
  return()
endif()

set(WB_HUMANOID_MPC_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../wb_humanoid_mpc" CACHE PATH "Path to the wb_humanoid_mpc checkout")
get_filename_component(WB_HUMANOID_MPC_ROOT "${WB_HUMANOID_MPC_ROOT}" ABSOLUTE)
set(WB_OCS2_ROOT "${WB_HUMANOID_MPC_ROOT}/lib/ocs2_ros2")
set(WB_NMPC_ROOT "${WB_HUMANOID_MPC_ROOT}/humanoid_nmpc")
if(NOT EXISTS "${WB_OCS2_ROOT}/ocs2_core/include/ocs2_core/Types.h" OR NOT EXISTS "${WB_NMPC_ROOT}/humanoid_wb_mpc")
  message(FATAL_ERROR
    "wb_humanoid_mpc not found at ${WB_HUMANOID_MPC_ROOT}. From the repository root run:\n"
    "  git submodule update --init src/wb_humanoid_mpc\n"
    "  git -C src/wb_humanoid_mpc submodule update --init lib/ocs2_ros2")
endif()
message(STATUS "wb_humanoid_mpc: ${WB_HUMANOID_MPC_ROOT}")

# ---- third-party dependencies -------------------------------------------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS filesystem log log_setup OPTIONAL_COMPONENTS system)
set(WB_BOOST_SYSTEM "")
if(TARGET Boost::system)
  set(WB_BOOST_SYSTEM Boost::system)
endif()
find_package(pinocchio REQUIRED)
find_package(urdfdom QUIET)

# ---- BLASFEO + HPIPM ------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(blasfeo
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d)
FetchContent_Declare(hpipm
  GIT_REPOSITORY https://github.com/giaf/hpipm
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BLASFEO_TESTING OFF CACHE BOOL "" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
set(HPIPM_FIND_BLASFEO OFF CACHE BOOL "" FORCE)
set(_wb_saved_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
# both projects read the CPU target from a cache variable called TARGET (set before each of them)
FetchContent_GetProperties(blasfeo)
if(NOT blasfeo_POPULATED)
  FetchContent_Populate(blasfeo)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(TARGET X64_AUTOMATIC CACHE STRING "" FORCE)
  else()
    set(TARGET GENERIC CACHE STRING "" FORCE)
  endif()
  add_subdirectory(${blasfeo_SOURCE_DIR} ${blasfeo_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
FetchContent_GetProperties(hpipm)
if(NOT hpipm_POPULATED)
  FetchContent_Populate(hpipm)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(TARGET AVX CACHE STRING "" FORCE)
  else()
    set(TARGET GENERIC CACHE STRING "" FORCE)
  endif()
  set(BLASFEO_INCLUDE_DIR "${blasfeo_SOURCE_DIR}/include" CACHE STRING "" FORCE)
  add_subdirectory(${hpipm_SOURCE_DIR} ${hpipm_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
set(BUILD_SHARED_LIBS ${_wb_saved_shared})
foreach(t blasfeo hpipm)
  set_target_properties(${t} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
endforeach()

# ---- compile settings shared by everything below ---------------------------------
set(WB_COMMON_OPTIONS -Wfatal-errors -fPIC -Wno-ignored-attributes -Wno-deprecated-declarations)
set(WB_COMMON_DEFINITIONS BOOST_ALL_DYN_LINK BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS BOOST_MPL_LIMIT_LIST_SIZE=30
                          BOOST_MPL_LIMIT_VECTOR_SIZE=30 PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

# Sources of the first add_library(${PROJECT_NAME} ...) call of an upstream (ament) CMakeLists.txt
function(wb_upstream_sources pkg_path out)
  file(READ "${pkg_path}/CMakeLists.txt" text)
  string(REGEX MATCH "add_library\\(\\$\\{PROJECT_NAME\\}[^)]*\\)" block "${text}")
  string(REGEX MATCHALL "src/[A-Za-z0-9_/.-]+\\.cpp" files "${block}")
  if(NOT files)
    message(FATAL_ERROR "no sources found in ${pkg_path}/CMakeLists.txt")
  endif()
  set(srcs "")
  foreach(f ${files})
    list(APPEND srcs "${pkg_path}/${f}")
  endforeach()
  set(${out} ${srcs} PARENT_SCOPE)
endfunction()

# wb_library(<name> <package dir> [EXCLUDE <regex...>] DEPS <targets...>): the upstream library, same sources
function(wb_library name pkg_path)
  cmake_parse_arguments(ARG "" "" "EXCLUDE;DEPS" ${ARGN})
  wb_upstream_sources("${pkg_path}" srcs)
  foreach(e ${ARG_EXCLUDE})
    list(FILTER srcs EXCLUDE REGEX "${e}")
  endforeach()
  set(target wbmpc_${name})
  add_library(${target} SHARED ${srcs})
  add_library(wbmpc::${name} ALIAS ${target})
  target_include_directories(${target} PUBLIC "${pkg_path}/include")
  target_compile_features(${target} PUBLIC cxx_std_20)
  target_compile_options(${target} PRIVATE ${WB_COMMON_OPTIONS})
  target_compile_definitions(${target} PUBLIC ${WB_COMMON_DEFINITIONS})
  target_link_libraries(${target} PUBLIC ${ARG_DEPS})
  set_target_properties(${target} PROPERTIES
    OUTPUT_NAME wbmpc_${name}
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
endfunction()

# ---- OCS2 fork ---------------------------------------------------------------------
wb_library(ocs2_core "${WB_OCS2_ROOT}/ocs2_core" 
  DEPS Eigen3::Eigen ${WB_BOOST_SYSTEM} Boost::filesystem Boost::log Boost::log_setup OpenMP::OpenMP_CXX Threads::Threads ${CMAKE_DL_LIBS})
target_include_directories(wbmpc_ocs2_core PUBLIC "${WB_OCS2_ROOT}/ocs2_thirdparty/include")
wb_library(ocs2_oc            "${WB_OCS2_ROOT}/ocs2_oc"            DEPS wbmpc::ocs2_core)
wb_library(ocs2_mpc           "${WB_OCS2_ROOT}/ocs2_mpc"           DEPS wbmpc::ocs2_oc)
wb_library(ocs2_qp_solver     "${WB_OCS2_ROOT}/ocs2_test_tools/ocs2_qp_solver" DEPS wbmpc::ocs2_oc)
wb_library(ocs2_ddp           "${WB_OCS2_ROOT}/ocs2_ddp"           DEPS wbmpc::ocs2_mpc wbmpc::ocs2_qp_solver)
wb_library(ocs2_robotic_tools "${WB_OCS2_ROOT}/ocs2_robotic_tools" DEPS wbmpc::ocs2_oc)
wb_library(hpipm_interface    "${WB_OCS2_ROOT}/ocs2_sqp/hpipm_catkin" DEPS wbmpc::ocs2_oc hpipm blasfeo)
wb_library(ocs2_sqp           "${WB_OCS2_ROOT}/ocs2_sqp/ocs2_sqp"  DEPS wbmpc::ocs2_mpc wbmpc::hpipm_interface)
wb_library(ocs2_pinocchio_interface "${WB_OCS2_ROOT}/ocs2_pinocchio/ocs2_pinocchio_interface" 
  DEPS wbmpc::ocs2_robotic_tools pinocchio::pinocchio)
if(TARGET urdfdom::urdfdom_model)
  target_link_libraries(wbmpc_ocs2_pinocchio_interface PUBLIC urdfdom::urdfdom_model)
endif()

# ---- humanoid NMPC -------------------------------------------------------------------
# DynamicsHelperFunctions.cpp is replaced by the fixed copy in src/wb_fixes (floating-base mass matrix, see the file)
wb_library(humanoid_common_mpc "${WB_NMPC_ROOT}/humanoid_common_mpc" 
  EXCLUDE "gait/GaitScheduleUpdater\\.cpp$" "reference_manager/ProceduralMpcMotionManager\\.cpp$"
          "pinocchio_model/DynamicsHelperFunctions\\.cpp$"
  DEPS wbmpc::ocs2_mpc wbmpc::ocs2_pinocchio_interface)
target_sources(wbmpc_humanoid_common_mpc PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../src/wb_fixes/DynamicsHelperFunctions.cpp")
target_include_directories(wbmpc_humanoid_common_mpc PRIVATE "${WB_NMPC_ROOT}/humanoid_common_mpc/test")

wb_library(humanoid_wb_mpc "${WB_NMPC_ROOT}/humanoid_wb_mpc" 
  EXCLUDE "mrt/WBMpcMrtJointController\\.cpp$"
  DEPS wbmpc::humanoid_common_mpc wbmpc::ocs2_sqp wbmpc::ocs2_ddp)
