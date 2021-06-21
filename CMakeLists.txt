# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(mrob)
#MESSAGE(Project\ name:\ ${PROJECT_NAME})


IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Release") 
ENDIF (NOT CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_EXTENSIONS ON)

IF(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") #TODO: /Wall but disable noisy warnings
    ADD_COMPILE_DEFINITIONS(_USE_MATH_DEFINES)
    SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
ELSE() #GCC or CLANG, but ...
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
ENDIF(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")



# ===================================================================
INCLUDE_DIRECTORIES(SYSTEM ./external/Eigen)

ADD_SUBDIRECTORY(./external/pybind11)


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


#INCLUDE_DIRECTORIES(./src/EigenFactors)
#ADD_SUBDIRECTORY(./src/EigenFactors)

# New modules should be included here

# MROB python bingins, defined here:
ADD_SUBDIRECTORY(./mrobpy)
