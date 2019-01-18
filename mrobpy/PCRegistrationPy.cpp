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
    // wrapper for handling EIgen::Ref, as the only way to pass by reference in pybind11
    ArunPy(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y) :
        Arun(X,Y) {};
};


SE3 ArunSolve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y)
{
    Arun a(X,Y);
    if (a.solve())
        std::cout << "no solution\n"; // TODO what if it fails?
    return a.getT();
}

class GicpPy : public Gicp
{
  public:
    GicpPy(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y,
           const py::EigenDRef<const MatX> covX, const py::EigenDRef<const MatX> covY ) :
        Gicp(X,Y,covX,covY) {};
};

void init_PCRegistration(py::module &m)
{
    py::class_<ArunPy>(m, "Arun")
            .def(py::init<py::EigenDRef<const MatX> , py::EigenDRef<const MatX> >())
            .def("solve", &ArunPy::solve)
            .def("getT", &ArunPy::getT)
            ;
    m.def("ArunSolve", &ArunSolve);
    py::class_<GicpPy>(m, "Gicp")
            .def(py::init<const py::EigenDRef<const MatX> , const py::EigenDRef<const MatX> ,
                    const py::EigenDRef<const MatX> , const py::EigenDRef<const MatX> >())
            .def("solve", &Gicp::solve)
            .def("getT", &Gicp::getT)
            ;
}


