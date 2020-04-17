/* Copyright 2018-2020 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * PCPlanesPy.cpp
 *
 *  Created on: Jan 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"


using namespace mrob;



void init_PCPlanes(py::module &m)
{
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
