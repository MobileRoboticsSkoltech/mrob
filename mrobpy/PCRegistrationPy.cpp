/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * PCRegistrationPy.cpp
 *
 *  Created on: Jan 17, 2019
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


#include "mrob/arun.hpp"
#include "mrob/gicp.hpp"


using namespace mrob;


class ArunPy : public Arun
{
  public:
    ArunPy(const py::EigenDRef<const MatX> &X, const py::EigenDRef<const MatX> &Y) :
        Arun(X,Y) {};
};


void init_PCRegistration(py::module &m)
{
    py::class_<ArunPy>(m, "Arun")
            .def(py::init<py::EigenDRef<const MatX> &, py::EigenDRef<const MatX> &>())
            .def("solve", &ArunPy::solve)
            .def("getT", &ArunPy::getT)
            ;
    py::class_<GICP>(m, "GICP")
            .def(py::init<const MatX &, const MatX &,
                          const MatX &, const MatX &>())
            .def("solve", &GICP::solve)
            .def("getT", &GICP::getT)
            ;
}


