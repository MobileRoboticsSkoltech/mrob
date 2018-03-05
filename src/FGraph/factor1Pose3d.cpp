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
             const Mat6 &obsCov):
             Factor(), dim_(6),  obs_(observation), Tobs_(observation),
             obsCov_(obsCov), J1_(Mat6::Zero()), r_(Mat61::Zero())
{
    neighbourNodes_.push_back(n1);
}

Factor1Pose3d::~Factor1Pose3d()
{
}

void Factor1Pose3d::evaluate()
{
    // Evaluate residual
    evaluateLazy();
    // Evaluate Jacobian
    J1_ = Mat6::Identity();
}

void Factor1Pose3d::evaluateLazy()
{
    r_ = Mat61::Identity();
}

void Factor1Pose3d::print() const
{
    std::cout << "Printing Anchor Factor of 1 Poses, obs= \n" <<
                    obs_ << "\nrepresenting the transformation\n" <<
                    Tobs_ << " \nand covariance\n" <<
                    obsCov_ << "\n and neighbour Nodes " <<
                    neighbourNodes_.size() << std::endl;
}
