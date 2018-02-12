# locate the necessary dependencies, if any

# extra source files
SET(sources
    NLOpt.cpp
)

# extra header files
SET(headers
    node.hpp
    factor.hpp
    NLOpt.hpp
)

# create the shared library
ADD_LIBRARY(NLOpt SHARED  ${sources})
#TARGET_LINK_LIBRARIES(NLopt )


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
#ADD_SUBDIRECTORY(test)
