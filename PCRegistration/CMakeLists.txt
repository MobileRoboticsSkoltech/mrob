# locate the necessary dependencies, if any


# extra source files
SET(sources
    src/base_transf.cpp
    src/create_points.cpp
    src/arun.cpp
    src/gicp.cpp
)

# extra header files
SET(headers
    base_transf.hpp
    create_points.cpp
    arun.hpp
    gicp.hpp
)

# create the shared library
ADD_LIBRARY(align_mth SHARED  ${sources})
TARGET_LINK_LIBRARIES(align_mth SE3)


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


ADD_SUBDIRECTORY(src/examples)
#ADD_SUBDIRECTORY(src/test)
