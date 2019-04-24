//
// Created by Konstantin on 14/01/2019.
//
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

void Factor1Pose2d::evaluate() {
    // Evaluate residual
    evaluate_residuals();
    chi2_ = r_.squaredNorm();


    // Evaluate Jacobian
    J_ = -Mat3::Identity();
}

void Factor1Pose2d::evaluate_residuals()
{
    r_ = get_neighbour_nodes()->at(0).get()->get_state() - obs_;
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


