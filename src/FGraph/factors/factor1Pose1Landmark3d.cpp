/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 * factor1Pose1Landmark3d.cpp
 *
 *  Created on: March 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor1Pose1Landmark3d.hpp"

#include <iostream>

using namespace mrob;


Factor1Pose1Landmark3d::Factor1Pose1Landmark3d(const Mat31 &observation, std::shared_ptr<Node> &nodePose,
        std::shared_ptr<Node> &nodeLandmark, const Mat3 &obsInf, bool initializeLandmark):
        Factor(3,9), obs_(observation), W_(obsInf), reversedNodeOrder_(false)
{
    // chek for order, we need to ensure id_0 < id_1
    if (nodePose->get_id() < nodeLandmark->get_id())
    {
        neighbourNodes_.push_back(nodePose);
        neighbourNodes_.push_back(nodeLandmark);
    }
    else
    {
        neighbourNodes_.push_back(nodeLandmark);
        neighbourNodes_.push_back(nodePose);
        // set reverse mode
        reversedNodeOrder_ = true;
    }

    if (initializeLandmark)
    {
        // TODO Initialize landmark value to whatever observation we see from current pose
    }
}

Factor1Pose1Landmark3d::~Factor1Pose1Landmark3d()
{
}

void Factor1Pose1Landmark3d::evaluate_residuals()
{
    // From T we observe z, and the residual is r = T^{-1}  landm - z
    uint_t poseIndex = 0; 
    uint_t landmarkIndex = 1;
    if (reversedNodeOrder_)
    {
        landmarkIndex = 0;
        poseIndex = 1; 
    }
    Mat4 Tx = get_neighbour_nodes()->at(poseIndex)->get_state();
    Tinv_ = SE3(Tx).inv();
    landmark_ = get_neighbour_nodes()->at(landmarkIndex)->get_state();
    r_ = Tinv_.transform(landmark_) - obs_;

}
void Factor1Pose1Landmark3d::evaluate_jacobians()
{
    // This function requries to FIRST evaluate residuals (which is always done)
    // dr / dT = d / dT ( T-1 Exp(-dx) l ) = T-1 [l^ -I]
    Mat<4,6> Jr = Mat<4,6>::Zero();
    Jr.topLeftCorner<3,3>() = hat3(landmark_);
    Jr.topRightCorner<3,3>() =  -Mat3::Identity();
    if (reversedNodeOrder_)
    {
        J_.topLeftCorner<3,3>() = Tinv_.R();
        J_.topRightCorner<3,6>() = ( Tinv_.T()* Jr).topLeftCorner<3,6>();
    }
    else
    {
        J_.topLeftCorner<3,6>() = ( Tinv_.T()* Jr).topLeftCorner<3,6>();
        J_.topRightCorner<3,3>() = Tinv_.R();   
    }
}

void Factor1Pose1Landmark3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor1Pose1Landmark3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

