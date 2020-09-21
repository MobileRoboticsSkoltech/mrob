# locate the necessary dependencies, if any

# extra header files
SET(headers
    mrob/time_profiling.hpp
    mrob/optimizer.hpp
)

# extra source files
SET(sources
    time_profiling.cpp
    optimizer.cpp
)
# create the shared library
ADD_LIBRARY(common SHARED  ${sources})
