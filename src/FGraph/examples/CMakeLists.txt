ADD_EXECUTABLE(example_FGraph  example_FGraph.cpp)
TARGET_LINK_LIBRARIES(example_FGraph FGraph)

ADD_EXECUTABLE(example_solver  example_solver.cpp)
TARGET_LINK_LIBRARIES(example_solver FGraph)

ADD_EXECUTABLE(example_FGraph2d example_FGraph_2d.cpp)
TARGET_LINK_LIBRARIES(example_FGraph2d FGraph)

ADD_EXECUTABLE(example_solver2d example_solver2d.cpp)
TARGET_LINK_LIBRARIES(example_solver2d FGraph)