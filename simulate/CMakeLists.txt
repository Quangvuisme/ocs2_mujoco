cmake_minimum_required(VERSION 3.16)
project(ocs2_mujoco)

# =============================
# Colored messages
# =============================
string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(ColourBold  "${Esc}[1m")
set(Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")
set(Blue        "${Esc}[34m")

macro(message_status text)
  message("-- ${Green}${text}${ColourReset}")
endmacro()

macro(message_warn text)
  message("-- ${Yellow}${text}${ColourReset}")
endmacro()

macro(message_error text)
  message(FATAL_ERROR "-- ${Red}${text}${ColourReset}")
endmacro()

# Enable languages
enable_language(C)
enable_language(CXX)

# Set standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================
# Optional CUDA support
# =============================
option(ENABLE_CUDA "Enable CUDA support for GPU point clouds" OFF)

# Set CUDA architectures before enabling the language
if(ENABLE_CUDA)
    set(CMAKE_CUDA_ARCHITECTURES "60;61;70;75;80;86;89;90" CACHE STRING "CUDA architectures")
endif()

if(ENABLE_CUDA)
    # Try to find CUDA
    find_package(CUDAToolkit QUIET)
    if(CUDAToolkit_FOUND)
        enable_language(CUDA)
        set(CMAKE_CUDA_STANDARD 17)
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
        set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
        message_status("CUDA support enabled")
        add_compile_definitions(CUDA_ENABLED)
    else()
        message_warn("CUDA not found, building without GPU support")
        set(ENABLE_CUDA OFF)
    endif()
else()
    message_status("CUDA disabled manually")
endif()


# =============================
# Optional ROS 2 support
# =============================
option(USE_ROS2 "Enable ROS 2 integration" OFF)

if(USE_ROS2)
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    find_package(sensor_msgs QUIET)
    find_package(geometry_msgs QUIET)
    find_package(image_transport QUIET)
    find_package(cv_bridge QUIET)
    find_package(tf2_ros QUIET)

    if(rclcpp_FOUND)
        message_status("ROS2 found: Building with ROS2 support")
        add_definitions(-DUSE_ROS2)
        set(ROS2_ENABLED TRUE)
    else()
        message_warn("ROS2 not found → Building without ROS2 support")
        set(ROS2_ENABLED FALSE)
    endif()
else()
    message_status("ROS2 disabled manually")
    set(ROS2_ENABLED FALSE)
endif()

# =============================
# Optional OpenCV support
# =============================
option(USE_OPENCV "Enable OpenCV integration" OFF)

if (USE_OPENCV)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        message_status("OpenCV found: building with OpenCV support")
        add_definitions(-DUSE_OPENCV)
        set(OPENCV_ENABLED TRUE)
    else()
        message_warn("OpenCV not found → Disabling OpenCV support")
        set(OPENCV_ENABLED FALSE)
    endif()
else()
    message_status("OpenCV disabled manually")
    set(OPENCV_ENABLED FALSE)
endif()

# =============================
# Dependencies
# =============================
find_package(mujoco REQUIRED)
find_package(OpenGL REQUIRED)



# Include directories
include_directories(
    "/usr/include/eigen3"
)

if(OPENCV_ENABLED)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# Add CUDA includes if available
if(ENABLE_CUDA AND CUDAToolkit_FOUND)
    include_directories(${CUDAToolkit_INCLUDE_DIRS})
endif()

# Collect source files
file(GLOB BASE_SRC
    src/mujoco/*.cc
)

file(GLOB ROS_SRC
    src/ros/*.cc
)

# Collect CUDA source files
file(GLOB CUDA_SRC
    src/ros/*.cu
)

set(SIM_SRC ${BASE_SRC})

if(ROS2_ENABLED)
    list(APPEND SIM_SRC ${ROS_SRC})
endif()

# =============================
# Create library
# =============================
# Dependencies
set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    OpenGL::GL
)

# Add CUDA libraries if available
if(ENABLE_CUDA AND CUDAToolkit_FOUND)
    list(APPEND SIM_DEPENDENCIES 
        CUDA::cudart 
        CUDA::cublas
        CUDA::curand
    )
endif()

add_link_options("LINKER:-as-needed")

# Create library with both C++ and CUDA sources
if(CUDA_SRC AND ENABLE_CUDA AND CUDAToolkit_FOUND)
    add_library(src ${SIM_SRC} ${CUDA_SRC})
    set_target_properties(src PROPERTIES 
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS ON
        CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
    )
else()
    add_library(src ${SIM_SRC})
endif()

# Link ROS dependencies if available
if(ROS2_ENABLED)
    ament_target_dependencies(src
        rclcpp
        sensor_msgs
        std_msgs
        geometry_msgs
        image_transport
        cv_bridge
        tf2_ros
    )
endif()

# =============================
# Executable
# =============================
add_executable(ocs2_mujoco ${SIM_SRC} src/main.cc)

# Set CUDA properties for the executable if available
if(ENABLE_CUDA AND CUDAToolkit_FOUND)
    set_target_properties(ocs2_mujoco PROPERTIES
        CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS ON
    )
endif()

# Link libraries
target_link_libraries(ocs2_mujoco 
    ${SIM_DEPENDENCIES}
    src
)

if(OPENCV_ENABLED)
    target_link_libraries(ocs2_mujoco ${OpenCV_LIBS})
endif()


if(ROS2_ENABLED)
    target_link_libraries(ocs2_mujoco tf2_ros::tf2_ros)
endif()

# =============================
# CUDA compile options
# =============================
if(ENABLE_CUDA AND CUDAToolkit_FOUND)
    target_compile_options(ocs2_mujoco PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:
            --use_fast_math
            -lineinfo
            --ptxas-options=-v
        >
    )
endif()

# Commented test executables
# add_executable(test test/test_unitree_sdk2.cpp)
# target_link_libraries(test unitree_sdk2)
# add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

set(CMAKE_BUILD_TYPE Release)

# Optional: Add CUDA runtime check
if(ENABLE_CUDA AND CUDAToolkit_FOUND)
    message_status("CUDA Version: ${CUDAToolkit_VERSION}")
    message_status("CUDA Architectures: ${CMAKE_CUDA_ARCHITECTURES}")
    message_status("CUDA Include Dirs: ${CUDAToolkit_INCLUDE_DIRS}")
else()
    message_status("Building without CUDA support - point clouds will use CPU")
endif()