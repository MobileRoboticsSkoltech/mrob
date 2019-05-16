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
    /**
     * Constructor for the python binding. By default uses the Cholesky adjoint solving type, and some estimated number of nodes and factors.
     */
    FGraphPy(uint_t potNumberNodes, uint_t potNumberFactors) : FGraphSolve(FGraphSolve::solveMethod::CHOL_ADJ,potNumberNodes,potNumberFactors) {};
    id_t add_node_pose_2d(const py::EigenDRef<const Mat31> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose2d(x));
        this->add_node(n);
        return n->get_id();
    }
    void add_factor_1pose_2d(const py::EigenDRef<const Mat31> obs, uint_t nodeId, const py::EigenDRef<const Mat3> obsInvCov)
    {
        auto n1 = this->get_node(nodeId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose2d(obs,n1,obsInvCov));
        this->add_factor(f);
    }
    void add_factor_2poses_2d(const py::EigenDRef<const Mat31> obs, uint_t nodeOriginId, uint_t nodeTargetId,
            const py::EigenDRef<const Mat3> obsInvCov, bool updateNodeTarget)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses2d(obs,nO,nT,obsInvCov, updateNodeTarget));
        this->add_factor(f);
    }
    void add_factor_2poses_2d_odom(const py::EigenDRef<const Mat31> obs, uint_t nodeOriginId, uint_t nodeTargetId, const py::EigenDRef<const Mat3> obsInvCov)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses2dOdom(obs,nO,nT,obsInvCov,true));//true is to update the node value according to obs
        this->add_factor(f);
    }

};



void init_FGraph(py::module &m)
{
    py::enum_<FGraphSolve::solveMethod>(m, "FGraph.solveMethod")
        .value("CHOL_ADJ", FGraphSolve::solveMethod::CHOL_ADJ)
        .value("CHOL", FGraphSolve::solveMethod::CHOL)
        .value("QR", FGraphSolve::solveMethod::QR)
        .value("SCHUR", FGraphSolve::solveMethod::SCHUR)
        .export_values()
        ;
    // Fgraph class adding factors and providing method to solve the inference problem.
    py::class_<FGraphPy> (m,"FGraph")
            .def(py::init<uint_t, uint_t>(),
                    "Constructor, solveType default is CHOL_ADJ. Second parameters are an estimated number of nodes and factors",
                    py::arg("potNumberNodes") = 512,
                    py::arg("potNumberFactors") = 512)
            //.def("set_solve_method", &FGraphSolve::set_solve_method)
            .def("solve_batch", &FGraphSolve::solve_batch)
            .def("solve_incremental", &FGraphSolve::solve_incremental)
            .def("chi2", &FGraphSolve::chi2)
            .def("get_estimated_state", &FGraphSolve::get_estimated_state)
            .def("add_node_pose_2d", &FGraphPy::add_node_pose_2d)
            .def("add_factor_1pose_2d", &FGraphPy::add_factor_1pose_2d)
            .def("add_factor_2poses_2d", &FGraphPy::add_factor_2poses_2d,
                    "Factors connecting 2 poses. If last input set to true (by default false), also updates the value of the target Node according to the new obs + origin node",
                    py::arg("obs"),
                    py::arg("nodeOridingId"),
                    py::arg("nodeTargetId"),
                    py::arg("obsInvCov"),
                    py::arg("updateNodeTarget") = false)
            .def("add_factor_2poses_2d_odom", &FGraphPy::add_factor_2poses_2d_odom)
            .def("print", &FGraph::print)
            ;
}
