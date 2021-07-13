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

#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"
#include "mrob/SE3Cov.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
namespace py = pybind11;
using namespace mrob;


void init_geometry(py::module &m) {
    py::class_<SE3>(m, "SE3")
		.def(py::init<>(),
				"The Default constructor creates the identity transformation",
				py::return_value_policy::copy)
        .def(py::init<const Mat4 &>(),
        		"Matrix constructor, requires a 4x4 RBT matrix",
				py::return_value_policy::copy)
        .def(py::init<const Mat61 &>(),
                "Given a vector xi in R^6, creates a RBT with the exponential mapping.",
                py::return_value_policy::copy)
        .def(py::init<const SE3 &>(),
                "Creates an SE3 from, copying another SE3",
                py::return_value_policy::copy)
        .def(py::init<const SO3 &, const Mat31>(),
                "Creates a RBT with a rotation object SO3 and a translation",
                py::return_value_policy::copy)
        .def("T", &SE3::T,
                "Outputs a 4x4 array with the RBT",
                py::return_value_policy::copy)
        .def("R",&SE3::R,
                "Outputs the Rotation array 3x3 component of the RBT",
                py::return_value_policy::copy)
        .def("t", &SE3::t,
                "Outputs the translation array 3D component of the RBT",
                py::return_value_policy::copy)
        .def("mul", &SE3::mul,
                "multiplies the current SE3 object by a second element. The order is: the right hand side of the current object times the second object",
                py::return_value_policy::copy)
        .def("update", &SE3::update_lhs,
                "updates the value of the SE3 by the tangent space coordinates provided. By default this is lhs")
        .def("update_lhs", &SE3::update_lhs,
                "updates the value of the SE3 by the tangent space coordinates provided. left hand sided")
        .def("update_rhs", &SE3::update_rhs,
                "updates the value of the SE3 by the tangent space coordinates provided. Right hand sided")
        .def("Ln", &SE3::ln_vee,
                "Logarithm + vee operator, returns 6D coordinates of the tangent space around the identity",
                py::return_value_policy::copy)
        .def("transform", &SE3::transform,
                "given a point in 3D, is transformed by the current RBT",
                py::return_value_policy::copy)
        .def("transform_array", &SE3::transform_array,
                "Given a stacked array of point Nx3, are transformed by the current RBT",
                py::return_value_policy::copy,
              "Input is a an array Nx3 and output is Nx3") // makes a copy of the array. TODO, pass by Ref and avoid copying, look at ownership
        .def("inv", &SE3::inv,
                "Outputs the inverse of the current SE3. This is a new object",
                py::return_value_policy::copy)
        .def("adj", &SE3::adj,
                "Returns the 6x6 adjoint matrix of the current SE3",
                py::return_value_policy::copy)
        .def("distance", &SE3::distance,
                "Calculates the distance between the current object and the argument as d = ||Ln(T^{1}*T_)||. If no element is provided, this is the norm of the object",
                py::arg("rhs")=SE3())
        .def("distance_rotation", &SE3::distance_rotation,
                "Calculates the rotation distance (of SO3 elements). If no element is provided this is just the element norm",
                py::arg("rhs")=SE3())
        .def("distance_trans", &SE3::distance_trans,
                "Calculates the translation distance. If no element is provided this is just the element norm",
                py::arg("rhs")=SE3())
        .def("print", &SE3::print, "Prints the current SE3 element")
        ;
    m.def("isSE3", &mrob::isSE3, "Returns True is the matrix is a valid transformation and False if not");

    py::class_<SO3>(m, "SO3")
		.def(py::init<>(), "Default SO3 construction, the identityt(3)", py::return_value_policy::copy)
		.def(py::init<const Mat31 &>(), "SO3 constructor with a 3D vector. Inside uses the exponential map",
		        py::return_value_policy::copy)
        .def(py::init<const Mat3 &>(), "SO3 constructor directly from an array 3x3", py::return_value_policy::copy)
        .def(py::init<const SO3 &>(), "Creates a new copy of the provided SO3", py::return_value_policy::copy)
        .def("R", &SO3::R,
                "Outputs the Rotation array 3x3 component",
                py::return_value_policy::copy)
        .def("mul", &SO3::mul,
                "multiplies a SO3 on the right hand side of the current object", py::return_value_policy::copy)
        .def("update", &SO3::update_lhs,
                "updates the value of the SO3 by the tangent space coordinates provided. By default this is lhs")
        .def("update_lhs", &SO3::update_lhs,
                "updates the value of the SO3 by the tangent space coordinates provided. left hand sided")
        .def("update_rhs", &SO3::update_rhs,
                "updates the value of the SO3 by the tangent space coordinates provided. right hand convention")
        .def("Ln", &SO3::ln_vee,
                "Logarithm operation, mapping to the tangent space around the identity element",
                py::return_value_policy::copy)
        .def("inv", &SO3::inv,
                "Creates a copy of the inverse",
                py::return_value_policy::copy)
        .def("adj", &SO3::adj,
                "Returns the 3x3 adjoint matrix of the SO3 element",
                py::return_value_policy::copy)
        .def("distance", &SO3::distance,
                "Calculates the distance between rotation matrices as ||Ln(R'*R_i)||")
        .def("print", &SO3::print, "Prints current information of the rotation")
        ;
    m.def("hat3", &mrob::hat3, "Returns a skew symmetric matrix 3x3 from a 3-vector", py::return_value_policy::copy);
    m.def("hat6", &mrob::hat6, "Returns a Lie algebra matrix 4x4 from a 6-vector", py::return_value_policy::copy);

    // AUxiliary functions to support other conventions (TORO, g2o)
    m.def("quat_to_so3", &quat_to_so3,"Suport function from quaternion to a rotation");
    m.def("so3_to_quat", &so3_to_quat,"Suport function from rotation matrix to quaternion");
    m.def("rpy_to_so3",  &rpy_to_so3,"Suport function from roll pitch yaw to a rotation");
        py::class_<SE3Cov, SE3>(m, "SE3Cov")
            .def(py::init<>(),
                 "Default construct a new SE3Cov object",
                 py::return_value_policy::copy)
            .def(py::init<const SE3 &, const Mat6 &>(),
                 "Construtor of SE3Cov object with given pose and covariance",
                 py::return_value_policy::copy)
            .def(py::init<const SE3Cov &>(),
                 "Copy constructor",
                 py::return_value_policy::copy)
            .def("cov", &SE3Cov::cov,
                 "returns current covariance matrix state",
                 py::return_value_policy::copy)
            .def("compound_2nd_order",
                 static_cast<void (SE3Cov::*)(const SE3Cov &)>(&SE3Cov::compound_2nd_order),
                 "Pose uncertainty compounding of the second order.")
            .def("compound_2nd_order",
                 static_cast<void (SE3Cov::*)(const SE3 &, const Mat6 &)>(&SE3Cov::compound_2nd_order),
                 "Pose uncertainty compounding of the second order.")
            .def("compound_4th_order",
                 static_cast<void (SE3Cov::*)(const SE3Cov &)>(&SE3Cov::compound_4th_order),
                 "SE3pose uncertainy compounding of the fourth order.")
            .def("compound_4th_order",
                 static_cast<void (SE3Cov::*)(const SE3 &, const Mat6 &)>(&SE3Cov::compound_4th_order),
                 "SE3pose uncertainy compounding of the fourth order.")
            .def("print",
                 &SE3Cov::print,
                 "Prints current state of pose and covariance.")
            .def("mul",
                 &SE3Cov::mul,
                 "Multiplication method mul() as an interface for compounding.",
                 py::return_value_policy::copy)
            .def("notation_transform",
                &SE3Cov::notation_transform,
                "Transforms covariance matrix to notation from Barfoot paper. Self-Inverse.",
                py::return_value_policy::copy);
}

