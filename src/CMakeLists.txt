# locate the necessary dependencies
find_package(Eigen QUIET)
if(Eigen_FOUND)
  MESSAGE("Eigen found in CMake modules:" ${Eigen_INCLUDE_DIRS})
ELSE(Eigen_FOUND)  
    SET(Eigen_INCLUDE_DIRS /usr/include/eigen3 /usr/local/include/eigen3)    
    MESSAGE("Eigen not found, adding manually directories. BE CAREFULL to have installed Eigen here:" ${Eigen_INCLUDE_DIRS})
    MESSAGE(${Eigen_INCLUDE_DIRS})
ENDIF(Eigen_FOUND)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

#Adding directories and source modules
INCLUDE_DIRECTORIES(./SE3)
INCLUDE_DIRECTORIES(./FGraph)
INCLUDE_DIRECTORIES(./align_mth)

# Adding source modules rules to the Cmake tree
ADD_SUBDIRECTORY(SE3)
ADD_SUBDIRECTORY(FGraph)
ADD_SUBDIRECTORY(align_mth)

