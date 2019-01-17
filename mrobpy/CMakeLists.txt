pybind11_add_module(mrob mrobPy.cpp SE3py.cpp PCRegistrationPy.cpp)
target_link_libraries(mrob PRIVATE SE3 PCRegistration)
