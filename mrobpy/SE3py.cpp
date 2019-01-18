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

#include "mrob/SO3.hpp"
#include "mrob/SE3.hpp"
using namespace mrob;


/**
 * Class to overcome the Eigen templated constructor, which does work.
 */
class PySO3 {
  public:
    PySO3(const Mat31 &w) : R_(w) { };
    PySO3(const Mat3 &R) : R_(R) { };
    Mat3 R() {return (Mat3)R_;};
    void update(const Mat31 &dw) {R_.update(dw);};
    Mat31 ln() {return R_.ln_vee();};
    PySO3 inv(){return PySO3(R_.inv());}
    Mat3 adj(){return R_.adj();}

  protected:
    SO3 R_;

};


/**
 * Class to overcome the Eigen templated constructor, which does work.
 */
class PySE3 {
  public:
    PySE3(const Mat61 &xi) : T_(xi) { };
    PySE3(const Mat4 &T) : T_(T) { };
    Mat4 T() {return (Mat4)T_;};
    Mat3 R() {return (Mat3)T_.R();}
    Mat31 t() {return T_.t();}
    void update(const Mat61 &dxi) {T_.update(dxi);};
    Mat61 ln() {return T_.ln_vee();};
    Mat31 transform(const Mat31 &p) {return T_.transform(p); }
    MatX transformArray(const MatX &p) {return T_.transformArray(p); }
    PySE3 inv(){return PySE3(T_.inv());}
    Mat6 adj(){return T_.adj();}

  protected:
    SE3 T_;

};



void init_SE3(py::module &m) {
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
}

