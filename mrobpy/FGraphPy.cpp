/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraphPy.cpp
 *
 *  Created on: Mar 15, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
namespace py = pybind11;


#include "mrob/factor_graph_solve.hpp"

using namespace mrob;

/**
 * This class facilitates the transition between the python binding and the Fgraph class.
 * In particular, we need to specify the class of factor/node and create a method on this
 * class and factors and nodes will be created here and added to the cpp structure.
 */
class FGraphPy: FGraphSolve
{
    uint_t add_node_2d(){};
    uint_t add_factor_2pose_2d(){};
    uint_t add_factor_1pose_2d(){};
};



void init_FGraph(py::module &m)
{
    py::class_<FGraphPy>(m,"FGraph")
            .def(py::init<uint_t,uint_t>())
            .def("addNode", &FGraphPy::add_node_2d)
            .def("addFactor", &FGraphPy::add_factor_2pose_2d)
            ;
}
