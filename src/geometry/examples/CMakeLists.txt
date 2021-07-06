IF (BUILD_TESTING)
    ADD_EXECUTABLE(SE3_example  example_SE3.cpp)
    TARGET_LINK_LIBRARIES(SE3_example SE3)

    ADD_EXECUTABLE(SE3_numerical_test  numerical_test.cpp)
    TARGET_LINK_LIBRARIES(SE3_numerical_test SE3)

    ADD_EXECUTABLE(test.SE3 test.SE3.cpp)
    TARGET_LINK_LIBRARIES(test.SE3 SE3 Catch2::Catch2)
ENDIF(BUILD_TESTING)
