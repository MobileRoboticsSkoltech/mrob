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

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

#include "skmr/SE3.hpp"
using namespace skmr;



/**
 * Class to overcome the templated constructor. Not working
 */
class PySE3 : public SE3{
  public:
    PySE3(const Mat61 &xi) : SE3(xi) { };
    Mat4 T() const {return (Mat4)*this;};
};


class MyClass{
    public:
    MyClass() : a_(10){};
    template <typename T> MyClass(T a) : a_(a) {};
    int get() {return a_;}
    protected:
    int a_;
};


PYBIND11_MODULE(skmrpy, m) {
    m.doc() = "pybind11 SE3 plugin";
    // Later, in binding code:
    py::class_<MyClass>(m, "SE3")
        .def(py::init<int>())
        .def("get",&MyClass::get)
        ;
}


/*
PYBIND11_MODULE(skmrpy, m) {
    m.doc() = "pybind11 SE3 plugin";
    // Later, in binding code:
    py::class_<PySE3>(m, "SE3")
        .def(py::init<const Mat61 &>())
        .def("T", &PySE3::T) // makes a copy of the 4x4 Transformation
        //.def("update", &SE3::update )
        //.def("ln_vee", &SE3::ln_vee)
        //.def("transform", &SE3::transform, py::return_value_policy::reference_internal)
        //.def("print",&PySE3::print)
        ;
}
*/
