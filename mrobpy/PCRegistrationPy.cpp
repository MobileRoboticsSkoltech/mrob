/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * PCRegistrationPy.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


/**
 * This program generates the Python binding for the SE3 library
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


SE3 arun_solve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y)
{
    SE3 res;
    PCRegistration::arun(X,Y,res);
    return res;
}


SE3 gicp_solve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y,
        const py::EigenDRef<const MatX> covX, const py::EigenDRef<const MatX> covY)
{
    SE3 res;
    PCRegistration::gicp(X,Y,covX,covY,res);
    return res;
}

SE3 weighted_solve(const py::EigenDRef<const MatX> X, const py::EigenDRef<const MatX> Y,
        const py::EigenDRef<const MatX1> weight)
{
    SE3 res;
    PCRegistration::weighted_point(X,Y,weight,res);
    return res;
}


void init_PCRegistration(py::module &m)
{
    m.def("align_arun", &arun_solve);
    m.def("align_gicp", &gicp_solve);
    m.def("align_weighted", &weighted_solve);
    py::class_<CreatePoints>(m,"CreatePoints")
            .def(py::init<uint_t, uint_t, uint_t, double>())
            .def("create_plane_registration", &CreatePoints::create_plane_registration)
            .def("get_point_cloud", &CreatePoints::get_point_cloud)
            .def("get_point_plane_ids", &CreatePoints::get_point_plane_ids)
            ;
    py::class_<PlaneRegistration>(m,"PlaneRegistration")
            .def(py::init<uint_t,uint_t>())
            .def("solve", &PlaneRegistration::solve_interpolate)
            .def("print", &PlaneRegistration::print)
            ;
}


