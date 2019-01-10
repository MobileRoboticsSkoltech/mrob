#INCLUDE_DIRECTORIES(.)

#TODO test this below, cange name to SE3 and do a python dinding for each module
pybind11_add_module(mrob SE3py.cpp)
target_link_libraries(mrob PRIVATE SE3)
