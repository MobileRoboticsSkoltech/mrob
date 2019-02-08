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

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
namespace py = pybind11;

#include "SE3py.hpp"




/*void init_SE3(py::module &m) {
    py::class_<PySE3>(m, "SE3")
        .def(py::init<const Mat61 &>())
        .def(py::init<const Mat4 &>())
        .def("T", &PySE3::T) // makes a copy of the 4x4 Transformation
        .def("R", &PySE3::R)
        .def("t", &PySE3::t)
        .def("update", &PySE3::update)
        .def("ln", &PySE3::ln)
        .def("transform", &PySE3::transform)
        .def("transformArray", &PySE3::transformArray)
        .def("inv", &PySE3::inv)
        .def("adj", &PySE3::adj)
        ;
    py::class_<PySO3>(m, "SO3")
        .def(py::init<const Mat31 &>())
        .def(py::init<const Mat3 &>())
        .def("R", &PySO3::R) // makes a copy of the 3x3 Transformation
        .def("update", &PySO3::update )
        .def("ln", &PySO3::ln)
        .def("inv", &PySO3::inv)
        .def("adj", &PySO3::adj)
        ;
}*/

void init_SE3(py::module &m) {
    py::class_<SE3>(m, "SE3")
        .def(py::init<const Mat61 &>())
        .def(py::init<const Mat4 &>())
        .def(py::init<const SE3 &>())
        .def("T", &SE3::T) // makes a copy of the 4x4 Transformation
        .def("R", &SE3::R)
        .def("t", &SE3::t)
        .def("update", &SE3::update)
        .def("ln", &SE3::ln_vee)
        .def("transform", &SE3::transform)
        .def("transformArray", &SE3::transformArray)
        .def("inv", &SE3::inv)
        .def("adj", &SE3::adj)
        ;
    py::class_<SO3>(m, "SO3")
        .def(py::init<const Mat31 &>())
        .def(py::init<const Mat3 &>())
        .def(py::init<const SO3 &>())
        .def("R", &SO3::R) // makes a copy of the 3x3 Transformation
        .def("update", &SO3::update )
        .def("ln", &SO3::ln_vee)
        .def("inv", &SO3::inv)
        .def("adj", &SO3::adj)
        ;
}

