cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

# init variables for convinience
set(OD_SOURCE_DIR ${OpenDetection_SOURCE_DIR})
set(OD_BINARY_DIR ${OpenDetection_BINARY_DIR})

# init directories before doing all the searches and includes
set(OD_CMAKE_DIR "${OpenDetection_SOURCE_DIR}/cmake" "${OpenDetection_SOURCE_DIR}/cmake/modules")
set(CMAKE_MODULE_PATH ${OD_CMAKE_DIR} ${CMAKE_MODULE_PATH})




# include bunch of stuffs and set the appropriate variables

# 1. Version
include(od_version)
set(OD_VERSION "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}")
set(OD_VERSION_DETAILED "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION}")
# generate config files
configure_file (
  "${OD_SOURCE_DIR}/ODConfig.h.in"
  "${OD_BINARY_DIR}/generated/ODConfig.h"
  )

# 3. set up different macros and configurations
include(od_targets)
include(od_mandatory_dependency)

# 2. collect config options and set appropriate variables which will be used in the next hierarchy
option(WITH_DOCUMENTATION "Build the OD documentation" ON)
option(WITH_GPU "Build GPU components of the project" ON)
option(WITH_SVMLIGHT "Build SVM Light" ON)

# 3. add all the modules
set(OD_MODULES_NAMES common doc detectors examples 3rdparty)
set(OD_MODULES_DIRS ${OD_MODULES_NAMES})

foreach(subdir ${OD_MODULES_DIRS})
  add_subdirectory("${OD_SOURCE_DIR}/${subdir}")
endforeach(subdir)





#include folders
include_directories("${OD_BINARY_DIR}/generated")




set(SOURCE_FILES opendetection.cpp)
add_executable(opendetection ${SOURCE_FILES})