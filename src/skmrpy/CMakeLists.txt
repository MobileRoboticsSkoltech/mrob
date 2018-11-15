#TODO test this below
pybind11_add_module(skmrpy SE3py.cpp)
target_link_libraries(skmrpy PRIVATE SE3)
