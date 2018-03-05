ADD_EXECUTABLE(test_FGraph  test_FGraph.cpp)
TARGET_LINK_LIBRARIES(test_FGraph FGraph)
ADD_EXECUTABLE(test_solver  test_solver.cpp)
TARGET_LINK_LIBRARIES(test_solver FGraph)