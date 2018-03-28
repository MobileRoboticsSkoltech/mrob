/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor1Pose3d.cpp
 *
 *  Created on: Mar 5, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "factor1Pose3d.hpp"
#include <iostream>

using namespace fg;


Factor1Pose3d::Factor1Pose3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsInf):
             Factor(6,6), obs_(observation), W_(obsInf)
{
    neighbourNodes_.push_back(n1);
    WT2_ = W_.llt().matrixU();// we get the upper matrix U'*U
}

Factor1Pose3d::~Factor1Pose3d()
{
}

void Factor1Pose3d::evaluate()
{
    // Evaluate residual
    evaluateError();
    // Evaluate Jacobian
}

matData_t Factor1Pose3d::evaluateError()
{
    r_ = Mat61::Identity();
    return 0.0;
}

void Factor1Pose3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= " << r_
              << " \nand covariance\n" << W_
              << "\n Calculated Jacobian = " << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}
