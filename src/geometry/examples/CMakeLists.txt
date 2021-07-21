IF (BUILD_TESTING)
    ADD_EXECUTABLE(SE3_numerical_test  numerical_test.cpp)
    TARGET_LINK_LIBRARIES(SE3_numerical_test SE3)

    ADD_EXECUTABLE(test.SE3 test.SE3.cpp)
    TARGET_LINK_LIBRARIES(test.SE3 SE3 Catch2::Catch2)
    ADD_TEST(test.SE3 bin/test.SE3)

    ADD_EXECUTABLE(test.SE3cov test.SE3cov.cpp)
    TARGET_LINK_LIBRARIES(test.SE3cov SE3 Catch2::Catch2)
    ADD_TEST(test.SE3cov bin/test.SE3cov)
ENDIF(BUILD_TESTING)
