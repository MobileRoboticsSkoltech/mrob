/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor2Poses3d.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor2Poses3d.hpp"

#include <iostream>
#include <Eigen/Cholesky>

using namespace mrob;


Factor2Poses3d::Factor2Poses3d(const Mat61 &observation, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf):
        Factor(6,12), obs_(observation), Tobs_(observation), W_(obsInf)
{
    assert(nodeOrigin->get_id() && "Factor2Poses3d::Factor2Poses3d: Non initialized Node1. Add nodes first and then Factors to the FG\n");
    assert(nodeTarget->get_id() && "Factor2Poses3d::Factor2Poses3d: Non initialized Node2. Add nodes first and then Factors to the FG\n");
    if (nodeOrigin->get_id() < nodeTarget->get_id())
    {
        neighbourNodes_.push_back(nodeOrigin);
        neighbourNodes_.push_back(nodeTarget);
    }
    else
    {
        neighbourNodes_.push_back(nodeTarget);
        neighbourNodes_.push_back(nodeOrigin);

        // inverse observations to correctly modify this
        obs_ = -observation;
    }
    WT2_ = W_.llt().matrixU();
}

Factor2Poses3d::~Factor2Poses3d()
{
}

void Factor2Poses3d::evaluate_residuals()
{
    // TODO Evaluation of residuals
    r_ = Mat61::Random();
}
void Factor2Poses3d::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    // TODO Jacobians
    J_ = Mat<6,12>::Random();
}

void Factor2Poses3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor2Poses3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

