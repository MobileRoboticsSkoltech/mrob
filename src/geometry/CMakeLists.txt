# locate the additional necessary dependencies, if any


# extra source files
SET(sources
    SO3.cpp
    SE3.cpp
    SE3cov.cpp
)

# extra header files
SET(headers
    mrob/SO3.hpp
    mrob/SE3.hpp
    mrob/SE3cov.hpp
)

# create the shared library
ADD_LIBRARY(SE3 SHARED  ${sources})
#TARGET_LINK_LIBRARIES( .. )
#target_link_libraries(${PROJECT_NAME} ${position_3d_LIBRARY})


#install 
#INSTALL(TARGETS people_prediction
#        RUNTIME DESTINATION bin
#        LIBRARY DESTINATION lib/SE3
#        ARCHIVE DESTINATION lib/SE3)

#INSTALL(FILES ${headers} DESTINATION include/iridrivers/people_prediction)
#INSTALL(FILES ${headers_random} DESTINATION include/iridrivers/people_prediction/random)
#INSTALL(FILES ${headers_scene_elements} DESTINATION include/iridrivers/people_prediction/scene_elements)
#INSTALL(FILES ${headers_nav} DESTINATION include/iridrivers/people_prediction/nav)


#INSTALL(FILES ../Findpeople_prediction.cmake DESTINATION ${CMAKE_ROOT}/Modules/)


ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(test)
