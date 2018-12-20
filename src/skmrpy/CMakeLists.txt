#TODO test this below
pybind11_add_module(mrob SE3py.cpp)
target_link_libraries(mrob PRIVATE SE3)
