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


#include "../src/PCRegistration/mrob/pc_registration.hpp"

#include "SE3py.hpp"

using namespace mrob;


PySE3 ArunSolve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y)
{
    SE3 res;
    PCRegistration::Arun(X,Y,res);
    return PySE3(res);
}


PySE3 GicpSolve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y,
        const py::EigenDRef<const MatX> covX, const py::EigenDRef<const MatX> covY)
{
    SE3 res;
    PCRegistration::Gicp(X,Y,covX,covY,res);
    return PySE3(res);
}

void init_PCRegistration(py::module &m)
{
    m.def("ArunSolve", &ArunSolve);
    m.def("GicpSolve", &GicpSolve);
}


