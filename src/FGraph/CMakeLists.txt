# locate the necessary dependencies, if any

# extra header files
SET(headers
    mrob/node.hpp
    mrob/factor.hpp
    mrob/factor_graph.hpp
    mrob/factor_graph_solve.hpp
)

# extra source files
SET(sources
    node.cpp
    factor.cpp
    factor_graph.cpp
    factor_graph_solve.cpp
)

SET(factors_headers
    mrob/factors/nodePose3d.hpp
    mrob/factors/factor2Poses3d.hpp
    mrob/factors/factor1Pose3d.hpp
    mrob/factors/nodePose2d.hpp
    mrob/factors/factor1Pose2d.hpp
    mrob/factors/factor2Poses2d.hpp
    mrob/factors/nodeLandmark2d.hpp
    mrob/factors/nodeLandmark3d.hpp
    mrob/factors/factor1Pose1Landmark2d.hpp
    mrob/factors/factor1Pose1Landmark3d.hpp
)

SET(factors_sources
    factors/nodePose3d.cpp
    factors/factor2Poses3d.cpp
    factors/factor1Pose3d.cpp
    factors/nodePose2d.cpp
    factors/factor1Pose2d.cpp
    factors/factor2Poses2d.cpp
    factors/nodeLandmark2d.cpp
    factors/nodeLandmark3d.cpp
    factors/factor1Pose1Landmark2d.cpp
    factors/factor1Pose1Landmark3d.cpp
)

# create the shared library
ADD_LIBRARY(FGraph SHARED  ${sources} ${factors_sources})
TARGET_LINK_LIBRARIES(FGraph SE3 common)


ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(tests)
