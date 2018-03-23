ADD_EXECUTABLE(example_FGraph  example_FGraph.cpp)
TARGET_LINK_LIBRARIES(example_FGraph FGraph)
ADD_EXECUTABLE(example_solver  example_solver.cpp)
TARGET_LINK_LIBRARIES(example_solver FGraph)
