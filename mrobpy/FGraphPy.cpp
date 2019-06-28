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
#include "mrob/factors/nodePose2d.hpp"
#include "mrob/factors/factor1Pose2d.hpp"
#include "mrob/factors/factor2Poses2d.hpp"
#include "mrob/factors/nodePose3d.hpp"
#include "mrob/factors/factor1Pose3d.hpp"
#include "mrob/factors/factor2Poses3d.hpp"


#include <Eigen/Geometry>

namespace py = pybind11;
using namespace mrob;



/**
 * Create auxiliary class to include all functions:
 *    - creates specific factors and nodes (while the cpp maintains a polymorphic data structure)
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

    // 3D factor graph
    id_t add_node_pose_3d(const py::EigenDRef<const Mat61> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose3d(x));
        this->add_node(n);
        return n->get_id();
    }
    void add_factor_1pose_3d(const py::EigenDRef<const Mat61> obs, uint_t nodeId, const py::EigenDRef<const Mat6> obsInvCov)
    {
        auto n1 = this->get_node(nodeId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose3d(obs,n1,obsInvCov));
        this->add_factor(f);
    }
    void add_factor_2poses_3d(const py::EigenDRef<const Mat61> obs, uint_t nodeOriginId, uint_t nodeTargetId,
            const py::EigenDRef<const Mat6> obsInvCov, bool updateNodeTarget)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses3d(obs,nO,nT,obsInvCov, updateNodeTarget));
        this->add_factor(f);
    }

};


/**
 * Function converting from quaternion q = [qx, qy, qz, qw](Eigen convention)
 * to a rotation matrix 3x3
 * XXX: ref eigen did not return a valid matrix (probably lifetime was managed from cpp and this object was local to this scope)
 */
Mat3 quat_to_so3(const py::EigenDRef<const Mat41> v)
{
    Eigen::Quaternion<matData_t> q(v);
    //std::cout << "Initial vector : " << v << ", transformed quaternion" << q.vec() << "\n and w = \n" << q.toRotationMatrix() << std::endl;
    return q.normalized().toRotationMatrix();
}

/**
 * Function converting from roll pitch yaw v = [r, p, y](Eigen convention)
 * to a rotation matrix 3x3
 */
Mat3 rpy_to_so3(const py::EigenDRef<const Mat31> v)
{
    Mat3 R;
    R = Eigen::AngleAxisd(v(0), Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(v(1), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(v(2), Eigen::Vector3d::UnitZ());
    return R;
}

void init_FGraph(py::module &m)
{
    py::enum_<FGraphSolve::solveMethod>(m, "FGraph.solveMethod")
        .value("CHOL_ADJ", FGraphSolve::solveMethod::CHOL_ADJ)
        .value("CHOL", FGraphSolve::solveMethod::CHOL)
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
            //.def("solve_incremental", &FGraphSolve::solve_incremental)//TODO not implemented yet
            .def("chi2", &FGraphSolve::chi2,
                    "Calculated the chi2 of the problem. By default re-evaluates residuals, set to false if doesn't",
                    py::arg("evaluateResidualsFlag") = true)
            .def("get_estimated_state", &FGraphSolve::get_estimated_state)
            .def("number_nodes", &FGraphSolve::number_nodes)
            .def("number_factors", &FGraphSolve::number_factors)
            .def("print", &FGraph::print)
            // -----------------------------------------------------------------------------
            // Specific call to 2D
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
            // -----------------------------------------------------------------------------
            // Specific call to 3D
            .def("add_node_pose_3d", &FGraphPy::add_node_pose_3d,
                    "Input are SE3 matrices directly")
            .def("add_factor_1pose_3d", &FGraphPy::add_factor_1pose_3d)
            .def("add_factor_2poses_3d", &FGraphPy::add_factor_2poses_3d,
                            "Factors connecting 2 poses. If last input set to true (by default false), also updates the value of the target Node according to the new obs + origin node",
                            py::arg("obs"),
                            py::arg("nodeOridingId"),
                            py::arg("nodeTargetId"),
                            py::arg("obsInvCov"),
                            py::arg("updateNodeTarget") = false)
            ;
        // AUxiliary functions to support other conventions (TORO, g2o)
        m.def("quat_to_so3", &quat_to_so3,"Suport function from quaternion to a rotation");
        m.def("rpy_to_so3",  &rpy_to_so3,"Suport function from roll pitch yaw to a rotation");
}
