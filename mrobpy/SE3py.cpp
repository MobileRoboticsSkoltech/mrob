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
#include <pybind11/stl.h>
namespace py = pybind11;

#include "mrob/SE3.hpp"
using namespace mrob;



/**
 * Class to overcome the Eigen templated constructor, which does work.
 */
class PySE3 {
  public:
    PySE3(const Mat61 &xi) : T_(xi) { };
    PySE3(const Mat4 &T) : T_(T) { };
    Mat4 T() {return (Mat4)T_;};
    void update(const Mat61 &dxi) {T_.update(dxi);};
    Mat61 ln() {return T_.ln_vee();};
    Mat31 transform(const Mat31 &p) {return T_.transform(p); }
    PySE3 inv(){return PySE3(T_.inv());}
    Mat6 adj(){return T_.adj();}

  protected:
    SE3 T_;

};



PYBIND11_MODULE(mrob, m) {
    m.doc() = "pybind11 SE3 plugin";
    // Later, in binding code:
    py::class_<PySE3>(m, "SE3")
        .def(py::init<const Mat61 &>())
        .def(py::init<const Mat4 &>())
        .def("T", &PySE3::T) // makes a copy of the 4x4 Transformation
        .def("update", &PySE3::update )
        .def("ln", &PySE3::ln)
        .def("transform", &PySE3::transform)
        .def("inv", &PySE3::inv)
        .def("adj", &PySE3::adj)
        ;
}

