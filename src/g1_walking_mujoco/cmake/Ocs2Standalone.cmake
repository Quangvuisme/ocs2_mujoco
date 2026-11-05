# =============================================================================
# Ocs2Standalone.cmake
#
# Builds the OCS2 libraries needed by a centroidal legged-robot MPC straight
# from the vendored sources in src/ocs2_ros2, with plain CMake: no ament, no
# colcon, no ROS. The upstream CMakeLists.txt files are ament packages, so they
# are not used; instead every library is compiled from the same src/*.cpp list
# (upstream adds every file under src/ to its library, so a glob is exact).
#
# Targets defined (all SHARED, guarded so several projects can include this):
#   ocs2::core  ocs2::oc  ocs2::mpc  ocs2::qp_solver  ocs2::ddp
#   ocs2::robotic_tools  ocs2::pinocchio_interface  ocs2::centroidal_model
#
# Inputs:
#   OCS2_ROOT  path to the ocs2_ros2 tree (default: ../ocs2_ros2)
# =============================================================================

if(TARGET ocs2_standalone_core)
  return()
endif()

set(OCS2_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../ocs2_ros2" CACHE PATH "Path to the vendored ocs2_ros2 sources")
get_filename_component(OCS2_ROOT "${OCS2_ROOT}" ABSOLUTE)
if(NOT EXISTS "${OCS2_ROOT}/core/ocs2_core/include/ocs2_core/Types.h")
  message(FATAL_ERROR "OCS2 sources not found at OCS2_ROOT=${OCS2_ROOT}")
endif()
message(STATUS "OCS2 sources: ${OCS2_ROOT}")

# ---- third-party dependencies -------------------------------------------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Boost.System is header-only since 1.69 and has no CMake component any more since 1.89 (e.g. the pip "pin" wheels)
find_package(Boost REQUIRED COMPONENTS filesystem log log_setup OPTIONAL_COMPONENTS system)
set(OCS2_BOOST_SYSTEM "")
if(TARGET Boost::system)
  set(OCS2_BOOST_SYSTEM Boost::system)
endif()
find_package(pinocchio REQUIRED)  # pinocchio 3.x CMake config (source / robotpkg / ROS / pip "pin")

# The generated CppAD code is compiled at runtime with gcc and loaded with dlopen
set(OCS2_COMMON_OPTIONS -Wfatal-errors -fPIC -Wno-ignored-attributes)
set(OCS2_COMMON_DEFINITIONS BOOST_ALL_DYN_LINK)

# ---- helper -------------------------------------------------------------------
# ocs2_standalone_library(<name> <package dir relative to OCS2_ROOT> DEPS <targets...>)
function(ocs2_standalone_library name pkg_dir)
  cmake_parse_arguments(ARG "" "" "DEPS" ${ARGN})
  set(pkg_path "${OCS2_ROOT}/${pkg_dir}")
  file(GLOB_RECURSE srcs CONFIGURE_DEPENDS "${pkg_path}/src/*.cpp")
  set(target ocs2_standalone_${name})
  add_library(${target} SHARED ${srcs})
  add_library(ocs2::${name} ALIAS ${target})
  target_include_directories(${target} PUBLIC "${pkg_path}/include")
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE ${OCS2_COMMON_OPTIONS})
  target_compile_definitions(${target} PUBLIC ${OCS2_COMMON_DEFINITIONS})
  target_link_libraries(${target} PUBLIC ${ARG_DEPS})
  set_target_properties(${target} PROPERTIES
    OUTPUT_NAME ocs2_${name}
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
endfunction()

# ---- libraries ------------------------------------------------------------------
ocs2_standalone_library(core core/ocs2_core
  DEPS Eigen3::Eigen ${OCS2_BOOST_SYSTEM} Boost::filesystem Boost::log Boost::log_setup
       OpenMP::OpenMP_CXX Threads::Threads ${CMAKE_DL_LIBS})
target_include_directories(ocs2_standalone_core PUBLIC "${OCS2_ROOT}/core/ocs2_thirdparty/include")

ocs2_standalone_library(oc              core/ocs2_oc                                       DEPS ocs2::core)
ocs2_standalone_library(mpc             mpc/ocs2_mpc                                       DEPS ocs2::oc)
ocs2_standalone_library(qp_solver       mpc/ocs2_qp_solver                                 DEPS ocs2::oc)
ocs2_standalone_library(ddp             mpc/ocs2_ddp                                       DEPS ocs2::mpc ocs2::qp_solver)
ocs2_standalone_library(robotic_tools   robotics/ocs2_robotic_tools                        DEPS ocs2::oc)
ocs2_standalone_library(pinocchio_interface robotics/ocs2_pinocchio/ocs2_pinocchio_interface DEPS ocs2::robotic_tools pinocchio::pinocchio)
ocs2_standalone_library(centroidal_model    robotics/ocs2_pinocchio/ocs2_centroidal_model DEPS ocs2::pinocchio_interface)

# Same flags the ROS build adds for pinocchio + urdfdom (see ocs2_centroidal_model/cmake/pinocchio_config.cmake)
# pinocchio's CMake target exports BOOST_MPL_LIMIT_{LIST,VECTOR}_SIZE=30 to every dependent translation
# unit; those limits only work with the non-preprocessed MPL headers (pinocchio/fwd.hpp sets this too,
# but files that use Boost without including pinocchio first would otherwise fail to compile).
foreach(t ocs2_standalone_pinocchio_interface ocs2_standalone_centroidal_model)
  target_compile_definitions(${t} PUBLIC PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
                                         BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
  target_compile_options(${t} PUBLIC -Wno-ignored-attributes)
endforeach()
