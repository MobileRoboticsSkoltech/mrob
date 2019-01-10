#Adding directories and source modules to the Cmake tree.


# SKMR modules. Remove those that are not necessary, by default all are active
INCLUDE_DIRECTORIES(./common)
#ADD_SUBDIRECTORY(./common) #only headers here

INCLUDE_DIRECTORIES(./SE3)
ADD_SUBDIRECTORY(./SE3)

INCLUDE_DIRECTORIES(./FGraph)
ADD_SUBDIRECTORY(./FGraph)

# TODO, when it is functional, it will be included
#INCLUDE_DIRECTORIES(./PCRegistration)
#ADD_SUBDIRECTORY(./PCRegistration)

# Adding external dependencies on the tree
SET(PYBIND11_CPP_STANDARD -std=c++11)
ADD_SUBDIRECTORY(./external/pybind11)

# Python bindings for the skmr library
INCLUDE_DIRECTORIES(./mrobpy)
ADD_SUBDIRECTORY(./mrobpy)
