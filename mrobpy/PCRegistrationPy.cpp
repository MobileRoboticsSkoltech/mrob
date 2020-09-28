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
namespace py = pybind11;


#include "mrob/SE3.hpp"
#include "mrob/pc_registration.hpp"


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
    m.def("arun", &arun_solve);
    m.def("gicp", &gicp_solve);
    m.def("weighted", &weighted_solve);
}


