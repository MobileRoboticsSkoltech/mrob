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
 * factor2Poses3d2obs.cpp
 *
 *  Created on: Nov 9, 2021
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor2Poses3d2obs.hpp"

#include <iostream>

using namespace mrob;


Factor2Poses3d2obs::Factor2Poses3d2obs(const Mat4 &observation, const Mat4 &observation2, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf,
        Factor::robustFactorType robust_type):
        Factor(6,12,robust_type), Tobs_(observation), Tobs2_(observation2), W_(obsInf)
{
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
        Tobs_.inv();
    }

}

Factor2Poses3d2obs::Factor2Poses3d2obs(const SE3 &observation, const SE3 &observation2, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf,
        Factor::robustFactorType robust_type):
        Factor(6,12, robust_type), Tobs_(observation),Tobs2_(observation2), W_(obsInf)
{
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
        Tobs_.inv();
    }

}


void Factor2Poses3d2obs::evaluate_residuals()
{
    // Tr =  T_o * T_obs * T_t * T_obs2^-1
    Mat4 TxOrigin = get_neighbour_nodes()->at(0)->get_state();
    Mat4 TxTarget = get_neighbour_nodes()->at(1)->get_state();
    Tr_ = SE3(TxOrigin) * Tobs_ * SE3(TxTarget) * Tobs2_.inv();
    r_ = Tr_.ln_vee();
    Tr_ = SE3(TxOrigin) * Tobs_;//This is the transformation needed later for the derivative
}
void Factor2Poses3d2obs::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals and  Tr (note Tr is not the exact residual but the required for the derivative)
    J_.topLeftCorner<6,6>() = Mat6::Identity();
    J_.topRightCorner<6,6>() = Tr_.adj();
}

void Factor2Poses3d2obs::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor2Poses3d2obs::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Tobs_.T()
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << ", " << neighbourNodes_[1]->get_id()
              << std::endl;
}

