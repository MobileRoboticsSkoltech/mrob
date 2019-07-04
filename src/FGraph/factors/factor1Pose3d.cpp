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


#include "mrob/factors/factor1Pose3d.hpp"

#include <iostream>
#include <Eigen/Cholesky>

using namespace mrob;


Factor1Pose3d::Factor1Pose3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsInf):
             Factor(6,6), obs_(observation), Tobs_(observation), W_(obsInf), J_(Mat6::Zero())
{
    // Ordering here is not a problem, the node is unique
    neighbourNodes_.push_back(n1);
    WT2_ = W_.llt().matrixU();// we get the upper matrix W = U'*U, and all elements get premultiplied by U = WT2
    //std::cout << "WT2 = \n" << WT2_ << std::endl;
}

Factor1Pose3d::~Factor1Pose3d()
{
}


void Factor1Pose3d::evaluate_residuals()
{
    // Anchor residuals as r = obs - x
    // r = ln(Tobs * x^-1) = - ln(X * Tobs^-1)
    // NOTE Tobs is a global observation (reference identity)
    Mat4 x = get_neighbour_nodes()->at(0).get()->get_stateT();
    Tr_ = Tobs_ * SE3(x).inv();
    r_ = Tr_.ln_vee();
}

void Factor1Pose3d::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SE3 and small perturbations TODO)
    // J = d/dxi ln(T X-1 exp(-xi) (T X-1)-1)= - Adj_{T X-1} = - Adj(Tr)
    J_ = -Tr_.adj();
}

void Factor1Pose3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1Pose3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}
