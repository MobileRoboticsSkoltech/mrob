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
#include "mrob/factors/factor1Pose2d.hpp"
#include "mrob/factors/factor2Poses2d.hpp"
#include "mrob/factors/nodePose2d.hpp"


namespace py = pybind11;
using namespace mrob;



/**
 * Create auxiliary class to include all functions creates factors and nodes
 *
 */

class FGraphPy : public FGraphSolve
{
public:
    id_t add_node_pose_2d(const py::EigenDRef<const Mat31> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose2d(x));
        this->add_node(n);
        return n->get_id();
    }

    void add_factor_2poses_2d(const py::EigenDRef<const Mat31> obs, uint_t nodeOriginId, uint_t nodeTargetId, const py::EigenDRef<const Mat3> obsInvCov)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses2d(obs,nO,nT,obsInvCov));
        this->add_factor(f);
    }

};



void init_FGraph(py::module &m)
{
    // Fgraph class adding factors and providing method to solve the inference problem.
    py::class_<FGraphPy>(m,"FGraph")
            .def(py::init<>())
            .def("add_node_pose_2d", &FGraphPy::add_node_pose_2d)
            .def("add_factor_2poses_2d", &FGraphPy::add_factor_2poses_2d)
            .def("print",&FGraph::print)
            ;
}
