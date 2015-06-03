cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# init variables for convinience
set(OD_SOURCE_DIR ${OpenDetection_SOURCE_DIR})
set(OD_BINARY_DIR ${OpenDetection_BINARY_DIR})

# init directories before doing all the searches and includes
set(OD_CMAKE_DIR "${OpenDetection_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${OD_CMAKE_DIR} ${CMAKE_MODULE_PATH})




# include bunch of stuffs and set the appropriate variables

# 1. Version
include(ODVersion)
set(OD_VERSION "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION}")
# generate config files
configure_file (
  "${OD_SOURCE_DIR}/ODConfig.h.in"
  "${OD_BINARY_DIR}/ODConfig.h"
  )











#include folders
include_directories("${OD_BINARY_DIR}")




set(SOURCE_FILES opendetection.cpp)
add_executable(opendetection ${SOURCE_FILES})