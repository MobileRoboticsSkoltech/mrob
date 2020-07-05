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
    py::enum_<PlaneRegistration::SolveMode>(m, "PlaneRegistration.SolveMethod")
        .value("INITIALIZE", PlaneRegistration::SolveMode::INITIALIZE)
        .value("GRADIENT", PlaneRegistration::SolveMode::GRADIENT)
        .value("GRADIENT_BENGIOS_NAG", PlaneRegistration::SolveMode::GRADIENT_BENGIOS_NAG)
        .value("GN_HESSIAN", PlaneRegistration::SolveMode::GN_HESSIAN)
        .value("LM_HESSIAN", PlaneRegistration::SolveMode::LM_HESSIAN)
        .value("GN_CLAMPED_HESSIAN", PlaneRegistration::SolveMode::GN_CLAMPED_HESSIAN)
        .value("LM_CLAMPED_HESSIAN", PlaneRegistration::SolveMode::LM_CLAMPED_HESSIAN)
        .export_values()
        ;
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
            .def("solve", &PlaneRegistration::solve,
                    py::arg("mode") = PlaneRegistration::SolveMode::GRADIENT,
                    py::arg("singleIteration") = false)
            .def("reset_solution", &PlaneRegistration::reset_solution, "resets the current solution and maintains the data from planes (PC)")
            .def("print", &PlaneRegistration::print,
                    py::arg("plotPlanes") =  false)
            .def("print_evaluate", &PlaneRegistration::print_evaluate,
                    "returns: current error,1) number of iters, 2) determinant 3) number of negative eigenvalues 4) conditioning number")
			// TODO add methods to fill in the data structure more properly, now it is a reference pass by sharing the smart pointer
            .def("get_point_cloud", &PlaneRegistration::get_point_cloud,
                    "Gets the point cloud at input time index")
            .def("get_number_poses", &PlaneRegistration::get_number_poses)
            .def("get_trajectory", &PlaneRegistration::get_trajectory)
            ;
}
