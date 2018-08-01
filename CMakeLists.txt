# rxKinFu: Moving Volume KinectFusion for 3D Perception and Robotics
#
# Author: Dimitrios Kanoulas (dkanoulas@gmail.com)

cmake_minimum_required(VERSION 2.8.12)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 OLD)
  cmake_policy(SET CMP0005 NEW)
  cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

# Project name
project(rxkinfu_pck)

# Set project version
set(RXKINFU_MAJOR_VERSION 1)
set(RXKINFU_MINOR_VERSION 0)
set(RXKINFU_PATCH_VERSION 0)
set(RXKINFU_VERSION
    ${RXKINFU_MAJOR_VERSION}.${RXKINFU_MINOR_VERSION}.${RXKINFU_PATCH_VERSION})

include(GNUInstallDirs)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

enable_testing()

# Whether to compile the SPL library
set(COMPILE_RXKINFU ON CACHE BOOL "Compile the rxkinfu lib")
set(COMPILE_RXKINFU_ROS ON CACHE BOOL "Compile the rxkinfu_ros")

# --------------------------------------
# Compile set of RXKINFU packages
# --------------------------------------
IF(COMPILE_RXKINFU)
  add_subdirectory(rxkinfu)
ENDIF(COMPILE_RXKINFU)

# --------------------------------------
# Compile set of RXKINFU_ROS packages
# --------------------------------------
IF(COMPILE_RXKINFU_ROS)
  add_subdirectory(rxkinfu_ros)
ENDIF(COMPILE_RXKINFU_ROS)
