/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

#include "mrob/plane.hpp"
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
        .value("GRADIENT_BENGIOS_NAG", PlaneRegistration::SolveMode::GRADIENT_BENGIOS_NAG)
        .value("GRADIENT_ALL_POSES", PlaneRegistration::SolveMode::GRADIENT_ALL_POSES)
        .value("GN_HESSIAN", PlaneRegistration::SolveMode::GN_HESSIAN)
        .value("GN_CLAMPED_HESSIAN", PlaneRegistration::SolveMode::GN_CLAMPED_HESSIAN)
        .value("LM_SPHER", PlaneRegistration::SolveMode::LM_SPHER)
        .value("LM_ELLIP", PlaneRegistration::SolveMode::LM_ELLIP)
        .export_values()
        ;
	// This class creates a synthetic testing
    py::class_<CreatePoints>(m,"CreatePoints")
            .def(py::init<uint_t, uint_t, uint_t, double, double>())
            .def("get_point_cloud", &CreatePoints::get_point_cloud,
            		"Input time index and outputs all points at that instant in time")
            .def("get_point_plane_ids", &CreatePoints::get_point_plane_ids,
            		"Input time index and outputs the plane IDs of each point, in the exact same order")
            .def("create_plane_registration", &CreatePoints::create_plane_registration,
            		"This fills in the structure for the class plane registration, ready to optimized with the synthetically created points. TODO this is a reference, not a deep copy!")
            .def("get_ground_truth_last_pose", &CreatePoints::get_ground_truth_last_pose,
                    "return SE3 of the last pose")
            .def("get_trajectory", &CreatePoints::get_ground_truth_pose)
            ;
    // This class is a data structure, containing all points and calculating plane registration
    py::class_<PlaneRegistration>(m,"PlaneRegistration")
            .def(py::init<>(),
                    "Constructor, by default empty structure")
            .def("set_number_planes_and_poses", &PlaneRegistration::set_number_planes_and_poses,
                    "sets the number of poses. Also deletes all planes and point inside.")
            .def("reset_solution", &PlaneRegistration::reset_solution,
                    "resets the current solution and maintains the data from planes (PC)")
            .def("solve", &PlaneRegistration::solve,
                    py::arg("mode") = PlaneRegistration::SolveMode::GRADIENT_BENGIOS_NAG,
                    py::arg("singleIteration") = false)
            .def("print", &PlaneRegistration::print,
                    py::arg("plotPlanes") =  false)
            .def("print_evaluate", &PlaneRegistration::print_evaluate,
                    "returns: current error,1) number of iters, 2) determinant 3) number of negative eigenvalues 4) conditioning number",
                    py::return_value_policy::copy)
			// TODO add methods to fill in the data structure more properly, now it is a reference pass by sharing the smart pointer
            .def("get_point_cloud", &PlaneRegistration::get_point_cloud,
                    "Gets the point cloud at input time index")
            .def("get_number_poses", &PlaneRegistration::get_number_poses)
            .def("get_trajectory", &PlaneRegistration::get_trajectory)
            .def("get_last_pose", &PlaneRegistration::get_last_pose)
            .def("add_plane", &PlaneRegistration::add_new_plane,
                    "input plane id (any integer and plane data structure")
            .def("plane_push_back_point", &PlaneRegistration::plane_push_back_point,
                    "input plane id and time id and 3 vector point")
            .def("get_error", &PlaneRegistration::get_current_error,
                    "get current error in plane estimation")
            .def("get_plane_error", &PlaneRegistration::get_current_error,
                    "input plane id (any integer and plane data structure")
            .def("initialize_last_pose_solution", &PlaneRegistration::set_last_pose,
                    "initializes the solution for some final input plane id (any integer and plane data structure")
            ;
}
