# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(mrob)
#MESSAGE(Project\ name:\ ${PROJECT_NAME})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX /usr/local)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Release") 
ENDIF (NOT CMAKE_BUILD_TYPE)

# flag _REENTRANT for safe threads and NDEBUG to disable asserts
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT -DNDEBUG")
#SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD 14)

# ===================================================================
# DEPENDENCIES: locate the necessary dependencies for the project
# DEPENDENCIES: Eigen
find_package(Eigen QUIET)
if(Eigen_FOUND)
  MESSAGE("Eigen found in CMake modules:" ${Eigen_INCLUDE_DIRS})
ELSE(Eigen_FOUND)  
    SET(Eigen_INCLUDE_DIRS /usr/local/include/eigen3;/usr/include/eigen3)
    MESSAGE("Eigen not found, adding manually directories. BE CAREFULL to have installed Eigen here:" ${Eigen_INCLUDE_DIRS})
    MESSAGE(${Eigen_INCLUDE_DIRS})
ENDIF(Eigen_FOUND)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

# DEPENDENCIES: pybind11
SET(PYBIND11_CPP_STANDARD -std=c++14)
ADD_SUBDIRECTORY(./pybind11)


# ===================================================================
# CODE: main modules and directories are included here

# MROB modules. Remove those that are not necessary, by default all are active
INCLUDE_DIRECTORIES(./src/common)
ADD_SUBDIRECTORY(./src/common)

INCLUDE_DIRECTORIES(./src/SE3)
ADD_SUBDIRECTORY(./src/SE3)

INCLUDE_DIRECTORIES(./src/FGraph)
ADD_SUBDIRECTORY(./src/FGraph)

INCLUDE_DIRECTORIES(./src/PCRegistration)
ADD_SUBDIRECTORY(./src/PCRegistration)


#INCLUDE_DIRECTORIES(./src/EigenFactors)
#ADD_SUBDIRECTORY(./src/EigenFactors)

# New modules should be included here

# MROB python bingins, defined here:
ADD_SUBDIRECTORY(./mrobpy)



ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)


