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


Factor2Poses3d::Factor2Poses3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
        std::shared_ptr<Node> &n2, const Mat6 &obsInf):
        Factor(6,12), obs_(observation), Tobs_(observation), W_(obsInf)
{
    assert(n1->get_id() && "Factor2Poses3d::Factor2Poses3d: Non initialized Node1. Add nodes first and then Factors to the FG\n");
    assert(n2->get_id() && "Factor2Poses3d::Factor2Poses3d: Non initialized Node2. Add nodes first and then Factors to the FG\n");
    if (n1->get_id() < n2->get_id())
    {
        neighbourNodes_.push_back(n1);
        neighbourNodes_.push_back(n2);
    }
    else
    {
        neighbourNodes_.push_back(n2);
        neighbourNodes_.push_back(n1);
    }
    WT2_ = W_.llt().matrixU();
}

Factor2Poses3d::~Factor2Poses3d()
{
}

void Factor2Poses3d::evaluate()
{
    // residuals
    this->evaluate_error();

    // TODO Jacobians
    J_ = Mat<6,12>::Random();
}
matData_t Factor2Poses3d::evaluate_error()
{
    // TODO Evaluation of residuals
    r_ = Mat61::Random();
    return 0.0;
}

void Factor2Poses3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= " << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = " << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

