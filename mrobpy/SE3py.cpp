/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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
        .def(py::init<const Mat61 &>(), py::return_value_policy::copy)
        .def(py::init<const Mat4 &>(), py::return_value_policy::copy)
        .def(py::init<const SE3 &>(), py::return_value_policy::copy)
        .def("T", &SE3::T, py::return_value_policy::copy) // makes a copy of the 4x4 Transformation
        .def("R", &SE3::R, py::return_value_policy::copy)
        .def("t", &SE3::t, py::return_value_policy::copy)
        .def("mul", &SE3::mul, "multiplies a SE3 on the right hand side of the current object", py::return_value_policy::copy)
        .def("update", &SE3::update_lhs)
        .def("update_lhs", &SE3::update_lhs)
        .def("update_rhs", &SE3::update_rhs)
        .def("ln", &SE3::ln_vee, py::return_value_policy::copy)
        .def("transform", &SE3::transform, py::return_value_policy::copy)
        .def("transform_array", &SE3::transform_array, py::return_value_policy::copy) // makes a copy of the array. TODO, pass by Ref and avoid copying, look at ownership
        .def("inv", &SE3::inv, py::return_value_policy::copy)
        .def("adj", &SE3::adj, py::return_value_policy::copy)
        .def("distance", &SE3::distance)
        .def("print", &SE3::print)
        ;
    m.def("isSE3", &mrob::isSE3, "Returns True is the matrix is a valid transformation and False if not");

    py::class_<SO3>(m, "SO3")
        .def(py::init<const Mat31 &>(), py::return_value_policy::copy)
        .def(py::init<const Mat3 &>(), py::return_value_policy::copy)
        .def(py::init<const SO3 &>(), py::return_value_policy::copy)
        .def("R", &SO3::R, py::return_value_policy::copy)
        .def("mul", &SO3::mul, "multiplies a SO3 on the right hand side of the current object", py::return_value_policy::copy)
        .def("update", &SO3::update_lhs )
        .def("update_lhs", &SO3::update_lhs )
        .def("update_rhs", &SO3::update_rhs )
        .def("ln", &SO3::ln_vee, py::return_value_policy::copy)
        .def("inv", &SO3::inv, py::return_value_policy::copy)
        .def("adj", &SO3::adj, py::return_value_policy::copy)
        .def("distance", &SO3::distance)
        .def("print", &SO3::print)
        ;
}

