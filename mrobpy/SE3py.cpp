/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * SE3py.cpp
 *
 *  Created on: Nov 15, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

/**
 * This programm generates the Pyhton binding for the SE3 library
 * using the pybind11 library, here included as an external dependency,
 */

#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
namespace py = pybind11;
using namespace mrob;

void init_SE3(py::module &m) {
    py::class_<SE3>(m, "SE3")
        .def(py::init<const Mat61 &>())
        .def(py::init<const Mat4 &>(), "don't print (python function) a directly created SE3 object with T(), it doesn't work well")
        .def(py::init<const SE3 &>())
        .def("T", &SE3::T) // makes a copy of the 4x4 Transformation
        .def("R", &SE3::R)
        .def("t", &SE3::t)
        .def("update", &SE3::update_lhs)
        .def("update_lhs", &SE3::update_lhs)
        .def("update_rhs", &SE3::update_rhs)
        .def("ln", &SE3::ln_vee)
        .def("transform", &SE3::transform)
        .def("transform_array", &SE3::transform_array)
        .def("inv", &SE3::inv)
        .def("adj", &SE3::adj)
        .def("print", &SE3::print)
        ;
    py::class_<SO3>(m, "SO3")
        .def(py::init<const Mat31 &>())
        .def(py::init<const Mat3 &>())
        .def(py::init<const SO3 &>())
        .def("R", &SO3::R) // makes a copy of the 3x3 Transformation
        .def("update", &SO3::update_lhs )
        .def("update_lhs", &SO3::update_lhs )
        .def("update_rhs", &SO3::update_rhs )
        .def("ln", &SO3::ln_vee)
        .def("inv", &SO3::inv)
        .def("adj", &SO3::adj)
        .def("print", &SO3::print)
        ;
}

