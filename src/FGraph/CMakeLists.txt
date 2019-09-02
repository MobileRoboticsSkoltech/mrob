# locate the necessary dependencies, if any

# extra header files
SET(headers
    mrob/node.hpp
    mrob/factor.hpp
    mrob/factor_graph.hpp
    mrob/factor_graph_solve.hpp
    mrob/CustomCholesky.hpp
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
    mrob/factors/factor2Poses2d.hpp)

SET(factors_sources
    factors/nodePose3d.cpp
    factors/factor2Poses3d.cpp
    factors/factor1Pose3d.cpp
    factors/nodePose2d.cpp
    factors/factor1Pose2d.cpp
    factors/factor2Poses2d.cpp)

# create the shared library
ADD_LIBRARY(FGraph SHARED  ${sources} ${factors_sources})
TARGET_LINK_LIBRARIES(FGraph SE3 common)


#install 
#INSTALL(TARGETS people_prediction
#        RUNTIME DESTINATION bin
#        LIBRARY DESTINATION lib/MRobotics
#        ARCHIVE DESTINATION lib/MRobotics)

#INSTALL(FILES ${headers} DESTINATION include/iridrivers/people_prediction)
#INSTALL(FILES ${headers_random} DESTINATION include/iridrivers/people_prediction/random)
#INSTALL(FILES ${headers_scene_elements} DESTINATION include/iridrivers/people_prediction/scene_elements)
#INSTALL(FILES ${headers_nav} DESTINATION include/iridrivers/people_prediction/nav)


#INSTALL(FILES ../Findpeople_prediction.cmake DESTINATION ${CMAKE_ROOT}/Modules/)


ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(tests)
