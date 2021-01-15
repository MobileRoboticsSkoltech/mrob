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
 * factor1PosePoint2Plane.cpp
 *
 *  Created on: Dec 24, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#include "mrob/factors/factor1PosePoint2Plane.hpp"

#include <iostream>


using namespace mrob;

Factor1PosePoint2Plane::Factor1PosePoint2Plane(const Mat31 &z_point_x, const Mat31 &z_point_y, const Mat31 &z_normal_y,
        std::shared_ptr<Node> &node,  const Mat1 &obsInf):
    Factor(1,6), z_point_x_(z_point_x), z_point_y_(z_point_y), z_normal_y_(z_normal_y), Tx_(Mat31::Zero()), r_(0.0), W_(obsInf)
{
    neighbourNodes_.push_back(node);
}


Factor1PosePoint2Plane::~Factor1PosePoint2Plane()
{
}


void Factor1PosePoint2Plane::evaluate_residuals()
{
    // r = <pi, Tp>
    Mat4 Tx = get_neighbour_nodes()->at(0)->get_state();
    SE3 T = SE3(Tx);
    Tx_ = T.transform(z_point_x_);
    r_ = Mat1(z_normal_y_.dot(Tx_ - z_point_y_));
}

void Factor1PosePoint2Plane::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    Mat<3,6> Jr;
    Jr << -hat3(Tx_) , Mat3::Identity();
    J_ = z_normal_y_.transpose() * Jr;
}

void Factor1PosePoint2Plane::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1PosePoint2Plane::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs point x= \n" << z_point_x_
              << "\nobs point y =\n" << z_point_y_
              << "\nobs normal y =\n" << z_normal_y_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << std::endl;
}
