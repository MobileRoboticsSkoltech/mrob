ADD_EXECUTABLE(SE3_example  example_SE3.cpp)
TARGET_LINK_LIBRARIES(SE3_example SE3)

ADD_EXECUTABLE(SE3_numerical_test  numerical_test.cpp)
TARGET_LINK_LIBRARIES(SE3_numerical_test SE3)