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
    factors/factor1PosePoint2Point.cpp
    factors/factor1PosePoint2Plane.cpp
    factors/nodePlane4d.cpp
    factors/factor1Pose1Plane4d.cpp
    factors/EigenFactorPlane.cpp
)

# extra header files
SET(headers
    mrob/pc_registration.hpp
    mrob/plane.hpp
    mrob/plane_registration.hpp
    mrob/create_points.hpp
    mrob/estimate_plane.hpp
    mrob/factors/factor1PosePoint2Point.hpp
    mrob/factors/factor1PosePoint2Plane.hpp
    mrob/factors/nodePlane4d.hpp
    mrob/factors/factor1Pose1plane4d.hpp
    mrob/factors/EigenFactorPlane.hpp
)

# create the shared library
ADD_LIBRARY(PCRegistration SHARED  ${sources} )
TARGET_LINK_LIBRARIES(PCRegistration SE3 FGraph common)


ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
