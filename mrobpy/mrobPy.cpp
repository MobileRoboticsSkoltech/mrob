/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * mrobPy.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <pybind11/pybind11.h>
namespace py = pybind11;



void init_geometry(py::module &m);
void init_FGraph(py::module &m);
void init_PCRegistration(py::module &m);
void init_PCPlanes(py::module &m);



PYBIND11_MODULE(mrob, m) {
    m.doc() = "pybind11 MROB library, now including \n-geometry: SE3, SO3 and other routines\n-registration: routines for PC aligment and others\n-fgrad: Factors Graphs ";
    // Later, in binding code:
    py::module m_geom = m.def_submodule("geometry");
    init_geometry(m_geom);

    py::module m_fg = m.def_submodule("fgraph");
    init_FGraph(m_fg);

    py::module m_reg = m.def_submodule("registration");
    init_PCRegistration(m_reg);
    init_PCPlanes(m_reg);
}




