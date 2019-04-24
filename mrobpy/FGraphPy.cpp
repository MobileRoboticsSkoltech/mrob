/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraphPy.cpp
 *
 *  Created on: Apr 5, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include "mrob/factor_graph_solve.hpp"


namespace py = pybind11;



void init_FGraph(py::module &m)
{
    // Define factors
    m.def("");
    // Fgraph class adding factors and providing method to solve the inference problem.
    py::class_<Fgraph>(m,"FGraph")
            .def(py::init<>())
            .def
            ;
}
