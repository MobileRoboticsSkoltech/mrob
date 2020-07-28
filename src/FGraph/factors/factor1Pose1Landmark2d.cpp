/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * factor1Pose1Landmark2d.cpp
 *
 *  Created on: Jul 27, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/factor1Pose1Landmark2d.hpp"

#include <iostream>

using namespace mrob;



Factor1Pose1Landmark2d::Factor1Pose1Landmark2d(const Mat21 &observation, std::shared_ptr<Node> &nodePose,
        std::shared_ptr<Node> &nodeLandmark, const Mat2 &obsInf, bool initializeLandmark):
        Factor(2,5), obs_(observation), r_(Mat21::Zero()),landmark_(Mat21::Zero()),
        state_(Mat31::Zero()),dx_(0.0), dy_(0.0), q_(0.0),
        W_(obsInf), reversedNodeOrder_(false)
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

Factor1Pose1Landmark2d::~Factor1Pose1Landmark2d()
{
}

void Factor1Pose1Landmark2d::evaluate_residuals()
{
    // check for order of pose landmark, it matter for Jacobian calculation
    uint_t poseIndex = 0;
    uint_t landmarkIndex = 1;
    if (reversedNodeOrder_)
    {
        landmarkIndex = 0;
        poseIndex = 1;
    }
    // From the local frame we observe the landmark
    state_ = get_neighbour_nodes()->at(poseIndex)->get_state();
    landmark_ = get_neighbour_nodes()->at(landmarkIndex)->get_state();
    dx_ = landmark_(0) - state_(0);
    dy_ = landmark_(1) - state_(1);
    q_ = dx_*dx_ + dy_*dy_;
    r_ << std::sqrt(q_),
         std::atan2(dy_,dx_) - state_(2);
    r_ -= obs_;
    r_(2) = wrap_angle(r_(2));

}
void Factor1Pose1Landmark2d::evaluate_jacobians()
{
    // This function requries to FIRST evaluate residuals (which is always done)
    Mat<2,3> Jx = Mat<2,3>::Zero();
    matData_t sqrt_q = std::sqrt(q_);
    Jx << -dx_/sqrt_q, -dy_/sqrt_q, 0,
           dy_/q_,    -dx_/q_,     -1;
    Mat<2,2> Jl = Mat<2,2>::Zero();
    Jl << dx_/sqrt_q, dy_/sqrt_q,
          -dy_/q_,    dx_/q_;
    if (!reversedNodeOrder_)
    {
        J_.topLeftCorner<2,3>() = Jx;
        J_.topRightCorner<2,2>() = Jl;
    }
    else
    {
        J_.topLeftCorner<2,2>() = Jl;
        J_.topRightCorner<2,3>() = Jx;
    }
}

void Factor1Pose1Landmark2d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor1Pose1Landmark2d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

