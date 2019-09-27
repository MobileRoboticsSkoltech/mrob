/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * plane_factor.cpp
 *
 *  Created on: Aug 16, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/plane_factor.hpp"

#include <iostream>

using namespace mrob;

PlaneFactor::PlaneFactor(const Mat4 &S, std::shared_ptr<Node> &nodeOrigin):
        Factor(0,0), //Dimension zero sinze this is a non-parametric factor. Also we don't known how many nodes will connect, so we set the second param to 0 (not-used)
        planeEstimation_(Mat41::Zero())
{
    neighbourNodes_.push_back(nodeOrigin);
    S_.emplace(nodeOrigin->get_id(), S);
}


void PlaneFactor::add_observation(const Mat4& S, std::shared_ptr<Node> &newNode)
{
    neighbourNodes_.push_back(newNode);
    S_.emplace(newNode->get_id(), S);
}


void PlaneFactor::print() const
{
    std::cout << "Plane Eigen Factor" <<std::endl;
}
