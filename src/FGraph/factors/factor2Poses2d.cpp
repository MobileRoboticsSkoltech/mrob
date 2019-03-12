//
// Created by Konstantin on 14/01/2019.
//


#include <iostream>
#include <Eigen/Cholesky>
#include <mrob/factors/factor2Poses2d.hpp>


using namespace mrob;


Factor2Poses2d::Factor2Poses2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                               std::shared_ptr<Node> &n2, const Mat3 &obsInf):
        Factor(3, 6), obs_(observation), W_(obsInf)
{
    assert(n1->getId() && "Factor2Poses2d::Factor2Poses2d: Non initialized Node1. Add nodes first and then Factors to the FG\n");
    assert(n2->getId() && "Factor2Poses2d::Factor2Poses2d: Non initialized Node2. Add nodes first and then Factors to the FG\n");
    if (n1->getId() < n2->getId())
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

void Factor2Poses2d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= " << r_
              << " \nand covariance\n" << W_
              << "\n Calculated Jacobian = " << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

double Factor2Poses2d::wrap_angle(double angle) {
    double pi2 = 2 * M_PI;

    while (angle < -M_PI) angle += pi2;
    while (angle >= M_PI) angle -= pi2;

    return angle;
}

