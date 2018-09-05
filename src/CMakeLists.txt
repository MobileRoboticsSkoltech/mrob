#Adding directories and source modules to the Cmake tree.
# Remove those that are not necessary, by default all are active
INCLUDE_DIRECTORIES(./common)
#ADD_SUBDIRECTORY(./common) #only headers here

INCLUDE_DIRECTORIES(./SE3)
ADD_SUBDIRECTORY(./SE3)

INCLUDE_DIRECTORIES(./FGraph)
ADD_SUBDIRECTORY(./FGraph)

INCLUDE_DIRECTORIES(./PCRegistration)
ADD_SUBDIRECTORY(./PCRegistration)
