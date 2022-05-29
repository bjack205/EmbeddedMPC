cmake_minimum_required(VERSION 3.18.0)
list(APPEND CMAKE_MESSAGE_CONTEXT empc)
project(EmbeddedMPC VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include useful modules
include(CMakePrintHelpers)
include(FetchContent)

##################################
# Dependencies
##################################

# GoogleTest
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()
include(GoogleTest)
include(CTest)

# fmtlib
include(FetchContent)
FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG d141cdbeb0fb422a3fb7173b285fd38e0d1772dc # version 8.0.1
)
FetchContent_MakeAvailable(fmt)

# Arduino Eigen
set(ARDUINO_LIB_DIR "${PROJECT_SOURCE_DIR}/deps")
add_library(ArduinoEigen INTERFACE)
target_include_directories(ArduinoEigen
  INTERFACE
  ${ARDUINO_LIB_DIR}/ArduinoEigen
)

#############################################
# Build
#############################################
set(OSQP_SOURCES
  ${PROJECT_SOURCE_DIR}/src/osqp/auxil.c
  ${PROJECT_SOURCE_DIR}/src/osqp/auxil.h
  ${PROJECT_SOURCE_DIR}/src/osqp/constants.h
  ${PROJECT_SOURCE_DIR}/src/osqp/error.c
  ${PROJECT_SOURCE_DIR}/src/osqp/error.h
  ${PROJECT_SOURCE_DIR}/src/osqp/glob_opts.h
  ${PROJECT_SOURCE_DIR}/src/osqp/kkt.c
  ${PROJECT_SOURCE_DIR}/src/osqp/kkt.h
  ${PROJECT_SOURCE_DIR}/src/osqp/lin_alg.c
  ${PROJECT_SOURCE_DIR}/src/osqp/lin_alg.h
  ${PROJECT_SOURCE_DIR}/src/osqp/osqp.c
  ${PROJECT_SOURCE_DIR}/src/osqp/osqp.h
  ${PROJECT_SOURCE_DIR}/src/osqp/osqp_configure.h
  ${PROJECT_SOURCE_DIR}/src/osqp/proj.c
  ${PROJECT_SOURCE_DIR}/src/osqp/proj.h
  ${PROJECT_SOURCE_DIR}/src/osqp/qdldl.c
  ${PROJECT_SOURCE_DIR}/src/osqp/qdldl.h
  ${PROJECT_SOURCE_DIR}/src/osqp/qdldl_interface.c
  ${PROJECT_SOURCE_DIR}/src/osqp/qdldl_interface.h
  ${PROJECT_SOURCE_DIR}/src/osqp/qdldl_types.h
  ${PROJECT_SOURCE_DIR}/src/osqp/qdldl_version.h
  ${PROJECT_SOURCE_DIR}/src/osqp/scaling.c
  ${PROJECT_SOURCE_DIR}/src/osqp/scaling.h
  ${PROJECT_SOURCE_DIR}/src/osqp/types.h
  ${PROJECT_SOURCE_DIR}/src/osqp/util.c
  ${PROJECT_SOURCE_DIR}/src/osqp/util.h
)

add_library(empc
  # src folder
  src/EmbeddedMPC.h
  src/osqp
  src/osqpsolver.cpp
  src/osqpsolver.hpp
  src/problem.cpp
  src/problem.hpp
  src/memory.cpp
  src/memory.hpp

  # osqp
  ${OSQP_SOURCES}
)
target_link_libraries(empc
  PUBLIC
  ArduinoEigen
)
target_include_directories(empc
  INTERFACE 
  ${PROJECT_SOURCE_DIR}/src
)

#############################################
# Tests
#############################################
add_subdirectory(test)