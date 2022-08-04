# locate the necessary dependencies, if any


# extra source files
SET(sources
    factors/factorCamera2d3dConstant.cpp
    factors/nodeCameraPinhole.cpp
)

# extra header files
SET(headers
    mrob/factors/factorCamera2d3dConstant.hpp
    mrob/factors/nodeCameraPinhole.hpp
)

# create the shared library
ADD_LIBRARY(VI SHARED  ${sources} )
TARGET_LINK_LIBRARIES(VI SE3 FGraph common)


#ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
