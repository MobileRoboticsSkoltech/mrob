# locate the necessary dependencies, if any

# extra header files
SET(headers
    mrob/plane_factor.hpp
)

# extra source files
SET(sources
    plane_factor.cpp
)

# create the shared library
ADD_LIBRARY(EigenFactors SHARED  ${sources})
TARGET_LINK_LIBRARIES(EigenFactors FGraph SE3)


ADD_SUBDIRECTORY(examples)
#ADD_SUBDIRECTORY(tests)
