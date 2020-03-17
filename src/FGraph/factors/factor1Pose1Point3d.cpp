/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * factor1Pose1Point3d.cpp
 *
 *  Created on: March 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor1Pose1Point3d.hpp"

#include <iostream>
#include <Eigen/Cholesky>

using namespace mrob;


Factor1Pose1Point3d::Factor1Pose1Point3d(const Mat31 &observation, std::shared_ptr<Node> &nodePose,
        std::shared_ptr<Node> &nodePoint, const Mat3 &obsInf):
        Factor(3,9), obs_(observation), W_(obsInf), reversedNodeOrder_(false)
{
    // XXX chek for order, we need to ensure id_0 < id_1
    if (nodePose->get_id() < nodePoint->get_id())
    {
        neighbourNodes_.push_back(nodePose);
        neighbourNodes_.push_back(nodePoint);
    }
    else
    {
        neighbourNodes_.push_back(nodePoint);
        neighbourNodes_.push_back(nodePose);
        // set reverse mode
        reversedNodeOrder_ = true;
    }

    WT2_ = W_.llt().matrixU();
}

Factor1Pose1Point3d::~Factor1Pose1Point3d()
{
}

void Factor1Pose1Point3d::evaluate_residuals()
{
    // From T we observe z, and the residual is r = T^{-1}  landm - z
    uint_t poseIndex = 0; 
    uint_t pointIndex = 1;
    if (reversedNodeOrder_)
    {
        pointIndex = 0;
        poseIndex = 1; 
    }
    Mat4 Tx = get_neighbour_nodes()->at(poseIndex)->get_stateT();
    Tinv_ = SE3(Tx).inv();
    landmark_ = get_neighbour_nodes()->at(pointIndex)->get_state();
    r_ = Tinv_.transform(landmark_) - obs_;

}
void Factor1Pose1Point3d::evaluate_jacobians()
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

void Factor1Pose1Point3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor1Pose1Point3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

