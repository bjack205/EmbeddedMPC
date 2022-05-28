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
add_library(empc
  # src folder
  src/EmbeddedMPC.h
  src/osqp
  src/osqpsolver.cpp
  src/osqpsolver.hpp
  src/problem.cpp
  src/problem.hpp

  # osqp
  src/osqp/auxil.c
  src/osqp/auxil.h
  src/osqp/constants.h
  src/osqp/error.c
  src/osqp/error.h
  src/osqp/glob_opts.h
  src/osqp/kkt.c
  src/osqp/kkt.h
  src/osqp/lin_alg.c
  src/osqp/lin_alg.h
  src/osqp/osqp.c
  src/osqp/osqp.h
  src/osqp/osqp_configure.h
  src/osqp/proj.c
  src/osqp/proj.h
  src/osqp/qdldl.c
  src/osqp/qdldl.h
  src/osqp/qdldl_interface.c
  src/osqp/qdldl_interface.h
  src/osqp/qdldl_types.h
  src/osqp/qdldl_version.h
  src/osqp/scaling.c
  src/osqp/scaling.h
  src/osqp/types.h
  src/osqp/util.c
  src/osqp/util.h
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