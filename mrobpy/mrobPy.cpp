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
 * mrobPy.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>


#include "mrob/optimizer.hpp"

namespace py = pybind11;



void init_geometry(py::module &m);
void init_FGraph(py::module &m);
void init_FGraphDense(py::module &m);
void init_PCRegistration(py::module &m);
void init_PCPlanes(py::module &m);



PYBIND11_MODULE(mrob, m) {
    m.doc() = "pybind11 MROB library, now including \n-geometry: SE3, SO3 and other routines\n-registration: routines for PC aligment and others\n-fgrad: Factors Graphs ";
    // Later, in binding code:
    py::add_ostream_redirect(m, "ostream_redirect");

    py::enum_<mrob::Optimizer::optimMethod>(m, "optimMethod")
        .value("NEWTON_RAPHSON", mrob::Optimizer::optimMethod::NEWTON_RAPHSON)
        .value("LEVENBERG_MARQUARDT_SPHER", mrob::Optimizer::optimMethod::LEVENBERG_MARQUARDT_SPHER)
        .value("LEVENBERG_MARQUARDT_ELLIP", mrob::Optimizer::optimMethod::LEVENBERG_MARQUARDT_ELLIP)
        .export_values()
        ;

    py::module m_geom = m.def_submodule("geometry");
    init_geometry(m_geom);

    py::module m_fg = m.def_submodule("fgraph");
    init_FGraph(m_fg);
    init_FGraphDense(m_fg);

    py::module m_reg = m.def_submodule("registration");
    init_PCRegistration(m_reg);
    init_PCPlanes(m_reg);
}




