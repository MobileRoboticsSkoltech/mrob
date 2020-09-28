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
 * factor2Poses3d.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factors/factor2Poses3d.hpp"

#include <iostream>

using namespace mrob;


Factor2Poses3d::Factor2Poses3d(const Mat4 &observation, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf, bool updateNodeTarget):
        Factor(6,12), Tobs_(observation), W_(obsInf)
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
    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        // carefull on the reference frame that Tobs is expressed at the X_origin frame, hence this change:
        Mat4 TxOrigin = nodeOrigin->get_state();
        nodeTarget->set_state( TxOrigin * Tobs_.T() );
    }
}

Factor2Poses3d::Factor2Poses3d(const SE3 &observation, std::shared_ptr<Node> &nodeOrigin,
        std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf, bool updateNodeTarget):
        Factor(6,12), Tobs_(observation), W_(obsInf)
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
    if (updateNodeTarget)
    {
        // Updates the child node such that it matches the odometry observation
        // carefull on the reference frame that Tobs is expressed at the X_origin frame, hence this change:
        Mat4 TxOrigin = nodeOrigin->get_state();
        nodeTarget->set_state( TxOrigin * Tobs_.T() );
    }
}

Factor2Poses3d::~Factor2Poses3d()
{
}

void Factor2Poses3d::evaluate_residuals()
{
    // From Origin we observe Target such that: T_o * T_obs = T_t
    // Tr = Txo * Tobs * Txt^-1
    // NOTE: We could also use the adjoint to refer the manifold coordinates obs w.r.t xo but in this case that
    // does not briung any advantage on the xo to the global frame (identity)
    // (xo reference)T_obs * T_xo  = T_xo * (global)T_obs. (rhs is what we use here)
    Mat4 TxOrigin = get_neighbour_nodes()->at(0)->get_state();
    Mat4 TxTarget = get_neighbour_nodes()->at(1)->get_state();
    Tr_ = SE3(TxOrigin) * Tobs_ * SE3(TxTarget).inv();
    r_ = Tr_.ln_vee();

}
void Factor2Poses3d::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    J_.topLeftCorner<6,6>() = Mat6::Identity();
    J_.topRightCorner<6,6>() = -Tr_.adj();
}

void Factor2Poses3d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}
void Factor2Poses3d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << Tobs_.T()
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

