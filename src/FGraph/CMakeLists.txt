# locate the necessary dependencies, if any

# extra header files
SET(headers
    node.hpp
    factor.hpp
    FGraph.hpp
    nodePose3d.hpp
    factor2Poses3d.hpp
    factor1Pose3d.hpp
    solverDense.hpp
)

# extra source files
SET(sources
    node.cpp
    factor.cpp
    FGraph.cpp
    nodePose3d.cpp
    factor2Poses3d.cpp
    factor1Pose3d.cpp
    solverDense.cpp
)

# create the shared library
ADD_LIBRARY(FGraph SHARED  ${sources})
TARGET_LINK_LIBRARIES(FGraph SE3)


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
ADD_SUBDIRECTORY(tests)
