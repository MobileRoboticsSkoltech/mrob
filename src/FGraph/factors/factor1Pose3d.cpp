/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

using namespace mrob;


Factor1Pose3d::Factor1Pose3d(const Mat4 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsInf):
             Factor(6,6), Tobs_(observation), W_(obsInf), J_(Mat6::Zero())
{
    // Ordering here is not a problem, the node is unique
    neighbourNodes_.push_back(n1);
}

Factor1Pose3d::Factor1Pose3d(const SE3 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsInf):
             Factor(6,6), Tobs_(observation), W_(obsInf), J_(Mat6::Zero())
{
    // Ordering here is not a problem, the node is unique
    neighbourNodes_.push_back(n1);
}

Factor1Pose3d::~Factor1Pose3d()
{
}


void Factor1Pose3d::evaluate_residuals()
{
    // Anchor residuals as r = x - obs
    // r = ln(X * Tobs^-1) = - ln(Tobs * x^-1)
    // NOTE Tobs is a global observation (reference identity)
    Mat4 x = get_neighbour_nodes()->at(0).get()->get_state();
    Tr_ = SE3(x) * Tobs_.inv();
    r_ = Tr_.ln_vee();
}

void Factor1Pose3d::evaluate_jacobians()
{
    // Evaluate Jacobian (see document on SE3 and small perturbations)
    // J = d/dxi ln(T X-1 exp(-xi) (T X-1)-1)= - Adj_{T X-1} = - Adj(Tr)
    // J = d/dxi ln(exp(xi)X T-1  (T X-1)-1)= I
    J_ = Mat6::Identity();
}

void Factor1Pose3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1Pose3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Tobs_.T()
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}
