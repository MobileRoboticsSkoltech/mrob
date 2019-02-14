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
#include <pybind11/stl.h>
namespace py = pybind11;


#include "mrob/SE3.hpp"
#include "mrob/pc_registration.hpp"
#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"


using namespace mrob;


SE3 ArunSolve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y)
{
    SE3 res;
    PCRegistration::Arun(X,Y,res);
    return res;
}


SE3 GicpSolve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y,
        const py::EigenDRef<const MatX> covX, const py::EigenDRef<const MatX> covY)
{
    SE3 res;
    PCRegistration::Gicp(X,Y,covX,covY,res);
    return res;
}

void init_PCRegistration(py::module &m)
{
    m.def("ArunSolve", &ArunSolve);
    m.def("GicpSolve", &GicpSolve);
    py::class_<CreatePoints>(m,"CreatePoints")
            .def(py::init<uint_t, uint_t, uint_t, double>())
            .def("get_point_cloud", &CreatePoints::get_point_cloud)
            .def("get_point_plane_ids", &CreatePoints::get_point_plane_ids)
            .def("create_plane_registration", &CreatePoints::create_plane_registration)
            ;
    py::class_<PlaneRegistration>(m,"PlaneRegistration")
            .def(py::init<uint_t,uint_t>())
            .def("solve", &PlaneRegistration::solve)
            .def("print", &PlaneRegistration::print)
            ;
}


