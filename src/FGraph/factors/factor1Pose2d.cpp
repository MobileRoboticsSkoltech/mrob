/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/factor1Pose2d.hpp"

#include <iostream>
#include <Eigen/Cholesky>

using namespace mrob;

Factor1Pose2d::Factor1Pose2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
        const Mat3 &obsInf) :
        Factor(3, 3), obs_(observation), W_(obsInf), J_(Mat3::Zero())
{
    neighbourNodes_.push_back(n1);
    WT2_ = W_.llt().matrixU();
}

void Factor1Pose2d::evaluate_jacobians()
{
    // Evaluate Jacobian
    J_ = -Mat3::Identity();
}

void Factor1Pose2d::evaluate_residuals()
{
    r_ = obs_ - get_neighbour_nodes()->at(0).get()->get_state();
}

void Factor1Pose2d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1Pose2d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}


