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



/**
 * Submodule dedicated to Point Clouds Plane aligment.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"


using namespace mrob;

/*PlaneRegistration CreatePoints::create_synthetic_plane_registration()
{
	PlaneRegistration data;

	return data;
}*/

void init_PCPlanes(py::module &m)
{
	// This class creates a synthetic testing
    py::class_<CreatePoints>(m,"CreatePoints")
            .def(py::init<uint_t, uint_t, uint_t, double>())
            .def("get_point_cloud", &CreatePoints::get_point_cloud,
            		"Input time index and outputs all points at that instant in time")
            .def("get_point_plane_ids", &CreatePoints::get_point_plane_ids,
            		"Input time index and outputs the plane IDs of each point, in the exact same order")
            .def("create_plane_registration", &CreatePoints::create_plane_registration,
            		"This fills in the structure for the class plane registration, ready to optimized with the synthetically created points. TODO this is a reference, not a deep copy!")
            ;
    // This class is a data structure, containing all points and calculating plane registration
    py::class_<PlaneRegistration>(m,"PlaneRegistration")
            .def(py::init<>(),
            		"Constructor, by default empty structure")
            .def("solve", &PlaneRegistration::solve_interpolate,
            		py::arg("singleIteration") = false)
            .def("print", &PlaneRegistration::print,
            		py::arg("plotPlanes") =  false)
			// TODO add methods to fill in the data structure more properly, now it is a reference pass by sharing the smart pointer
            ;
}
