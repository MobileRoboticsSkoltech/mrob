# locate the necessary dependencies, if any


# extra source files
SET(sources
    arun.cpp
    gicp.cpp
    plane.cpp
    plane_registration.cpp
    create_points.cpp
    weight_point.cpp
    estimate_plane.cpp
    factors/factor1PosePoint2Plane.cpp
)

# extra header files
SET(headers
    mrob/pc_registration.hpp
    mrob/plane.hpp
    mrob/plane_registration.hpp
    mrob/create_points.hpp
    mrob/estimate_plane.hpp
    mrob/factors/factor1PosePoint2Plane.hpp
)

# create the shared library
ADD_LIBRARY(PCRegistration SHARED  ${sources} )
TARGET_LINK_LIBRARIES(PCRegistration SE3 FGraph common)


ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
