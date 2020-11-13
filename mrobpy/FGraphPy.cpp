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
#include "mrob/factors/nodeLandmark3d.hpp"
#include "mrob/factors/factor1Pose1Landmark3d.hpp"
#include "mrob/factors/nodeLandmark2d.hpp"
#include "mrob/factors/factor1Pose1Landmark2d.hpp"
#include "mrob/factors/nodePlane4d.hpp"
#include "mrob/factors/factor1Pose1Plane4d.hpp"

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
    FGraphPy() :
        FGraphSolve(FGraphSolve::matrixMethod::ADJ,
                    FGraphSolve::optimMethod::GN) {};
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
    // ------------------------------------------------------------------------------------
    /*id_t add_node_pose_3d(const py::EigenDRef<const Mat4> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose3d(x));
        this->add_node(n);
        return n->get_id();
    }*/
    id_t add_node_pose_3d(const SE3 &x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePose3d(x));
        this->add_node(n);
        return n->get_id();
    }
    id_t add_factor_1pose_3d(const SE3 &obs, uint_t nodeId, const py::EigenDRef<const Mat6> obsInvCov)
    {
        auto n1 = this->get_node(nodeId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose3d(obs,n1,obsInvCov));
        this->add_factor(f);
        return f->get_id();
    }
    id_t add_factor_2poses_3d(const SE3 &obs, uint_t nodeOriginId, uint_t nodeTargetId,
            const py::EigenDRef<const Mat6> obsInvCov, bool updateNodeTarget)
    {
        auto nO = this->get_node(nodeOriginId);
        auto nT = this->get_node(nodeTargetId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses3d(obs,nO,nT,obsInvCov, updateNodeTarget));
        this->add_factor(f);
        return f->get_id();
    }
    
    // 3D Landmarks
    // ------------------------------------------------------------------------------------
    id_t add_node_landmark_3d(const py::EigenDRef<const Mat31> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodeLandmark3d(x));
        this->add_node(n);
        return n->get_id();
    }
    id_t add_factor_1pose_1landmark_3d(const py::EigenDRef<const Mat31> obs, uint_t nodePoseId,
                uint_t nodeLandmarkId, const py::EigenDRef<const Mat3> obsInvCov, bool initializeLandmark)
    {
        auto n1 = this->get_node(nodePoseId);
        auto n2 = this->get_node(nodeLandmarkId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose1Landmark3d(obs,n1,n2,obsInvCov,initializeLandmark));
        this->add_factor(f);
        return f->get_id();
    }


    // 2D Landmarks
    // ------------------------------------------------------------------------------------
    id_t add_node_landmark_2d(const py::EigenDRef<const Mat21> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodeLandmark2d(x));
        this->add_node(n);
        return n->get_id();
    }
    id_t add_factor_1pose_1landmark_2d(const py::EigenDRef<const Mat21> obs, uint_t nodePoseId,
                uint_t nodeLandmarkId, const py::EigenDRef<const Mat2> obsInvCov, bool initializeLandmark)
    {
        auto n1 = this->get_node(nodePoseId);
        auto n2 = this->get_node(nodeLandmarkId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose1Landmark2d(obs,n1,n2,obsInvCov,initializeLandmark));
        this->add_factor(f);
        return f->get_id();
    }

    // Planes in 4d, i.e. pi = [nx.ny,nz,d] \in P^3
    // There is a problem with scaling of long distances (d->inf n->0), but well, this
    // is not a minimal representation. For finite distances should be fine.
    // ------------------------------------------------------------------------------------
    id_t add_node_plane_4d(const py::EigenDRef<const Mat41> x)
    {
        std::shared_ptr<mrob::Node> n(new mrob::NodePlane4d(x));
        this->add_node(n);
        return n->get_id();
    }
    id_t add_factor_1pose_1plane_4d(const py::EigenDRef<const Mat41> obs, uint_t nodePoseId,
                uint_t nodeLandmarkId, const py::EigenDRef<const Mat4> obsInvCov)
    {
        auto n1 = this->get_node(nodePoseId);
        auto n2 = this->get_node(nodeLandmarkId);
        std::shared_ptr<mrob::Factor> f(new mrob::Factor1Pose1Plane4d(obs,n1,n2,obsInvCov));
        this->add_factor(f);
        return f->get_id();
    }

};

void init_FGraph(py::module &m)
{
    py::enum_<FGraphSolve::optimMethod>(m, "FGraph.optimMethod")
        .value("GN", FGraphSolve::optimMethod::GN)
        .value("LM", FGraphSolve::optimMethod::LM)
        .export_values()
        ;
    // Fgraph class adding factors and providing method to solve the inference problem.
    py::class_<FGraphPy> (m,"FGraph")
            .def(py::init<>(),
                    "Constructor, solveType default is ADJ and GN.")
            .def("solve", &FGraphSolve::solve,
                    "Solves the corresponding FG",
                    py::arg("method") =  FGraphSolve::optimMethod::GN,
                    py::arg("maxIters") = 30)
            .def("chi2", &FGraphSolve::chi2,
                    "Calculated the chi2 of the problem. By default re-evaluates residuals, set to false if doesn't",
                    py::arg("evaluateResidualsFlag") = true)
            .def("get_estimated_state", &FGraphSolve::get_estimated_state,
                    "returns the list of states ordered according to ids. Some of these elements might be matrices if the are 3D poses")
            .def("get_information_matrix", &FGraphSolve::get_information_matrix,
                    "Returns the information matrix. It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("get_chi2_array", &FGraphSolve::get_chi2_array,
                    "Returns the vector of chi2 values for each factor. It requires to be calculated -> solved the problem",
                    py::return_value_policy::copy)
            .def("number_nodes", &FGraphSolve::number_nodes, "Returns the number of nodes")
            .def("number_factors", &FGraphSolve::number_factors, "Returns the number of factors")
            .def("print", &FGraph::print, "By default False: does not print all the information on the Fgraph", py::arg("completePrint") = false)
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
            // 2d Landmkarks
            .def("add_node_landmark_2d", &FGraphPy::add_node_landmark_2d,
                    "Ladmarks are 2D points, in [x,y]")
            .def("add_factor_1pose_1landmark_2d", &FGraphPy::add_factor_1pose_1landmark_2d,
                            "Factor connecting 1 pose and 1 point (landmark).",
                            py::arg("obs"),
                            py::arg("nodePoseId"),
                            py::arg("nodeLandmarkId"),
                            py::arg("obsInvCov"),
                            py::arg("initializeLandmark") = false)
            // -----------------------------------------------------------------------------
            // Specific call to 3D
            .def("add_node_pose_3d", &FGraphPy::add_node_pose_3d,
                    "Input are 3D poses, as Lie Algebra of RBT around the Identity")
            .def("add_factor_1pose_3d", &FGraphPy::add_factor_1pose_3d)
            .def("add_factor_2poses_3d", &FGraphPy::add_factor_2poses_3d,
                            "Factors connecting 2 poses. If last input set to true (by default false), also updates the value of the target Node according to the new obs + origin node",
                            py::arg("obs"),
                            py::arg("nodeOridingId"),
                            py::arg("nodeTargetId"),
                            py::arg("obsInvCov"),
                            py::arg("updateNodeTarget") = false)
            // -----------------------------------------------------------------------------
            // Landmark or Point 3D
            .def("add_node_landmark_3d", &FGraphPy::add_node_landmark_3d,
                    "Ladmarks are 3D points, in [x,y,z]")
            .def("add_factor_1pose_1landmark_3d", &FGraphPy::add_factor_1pose_1landmark_3d,
                            "Factor connecting 1 pose and 1 point (landmark).",
                            py::arg("obs"),
                            py::arg("nodePoseId"),
                            py::arg("nodeLandmarkId"),
                            py::arg("obsInvCov"),
                            py::arg("initializeLandmark") = false)
            // -----------------------------------------------------------------------------
            // Plane 4d Landmark to Pose 3D
            .def("add_node_plane_4d", &FGraphPy::add_node_plane_4d,
                    "Panes are points in P^3, in [nx,ny,nz, d]")
            .def("add_factor_1pose_1plane_4d", &FGraphPy::add_factor_1pose_1plane_4d,
                            "Factor observing a plane(landmark) from the current pose.",
                            py::arg("obs"),
                            py::arg("nodePoseId"),
                            py::arg("nodeLandmarkId"),
                            py::arg("obsInvCov"))
            ;

}
