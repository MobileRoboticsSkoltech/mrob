# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(mrob)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Release") 
ENDIF (NOT CMAKE_BUILD_TYPE)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_EXTENSIONS ON)

IF(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") #TODO: /Wall but disable noisy warnings
    ADD_COMPILE_DEFINITIONS(_USE_MATH_DEFINES)
    SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
ELSEIF(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
ELSE()
ENDIF(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")

IF(ANDROID)
    SET(BUILD_TESTING OFF)
ELSE(ANDROID)
    SET(BUILD_TESTING ON)
    enable_testing()
ENDIF(ANDROID)

MESSAGE(STATUS "Build Tests: ${BUILD_TESTING}")

# ===================================================================
INCLUDE_DIRECTORIES(SYSTEM ./external/Eigen)

# ===================================================================
# CODE: main modules and directories are included here

# MROB modules. Remove those that are not necessary, by default all are active
INCLUDE_DIRECTORIES(./src/common)
ADD_SUBDIRECTORY(./src/common)

INCLUDE_DIRECTORIES(./src/geometry)
ADD_SUBDIRECTORY(./src/geometry)

INCLUDE_DIRECTORIES(./src/FGraph)
ADD_SUBDIRECTORY(./src/FGraph)

INCLUDE_DIRECTORIES(./src/PCRegistration)
ADD_SUBDIRECTORY(./src/PCRegistration)



# ===================================================================
# New modules should be included here

IF(NOT ANDROID)
    ADD_SUBDIRECTORY(./external/pybind11)
    ADD_SUBDIRECTORY(./mrobpy)
ENDIF(NOT ANDROID)
