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

Factor1PosePoint2Plane::Factor1PosePoint2Plane(const Mat31 &z_point, const Mat41 &z_plane,  std::shared_ptr<Node> &node,
        const Mat1 &obsInf):
    Factor(6,1), z_point_(z_point), z_plane_(z_plane), Tpoint_(Mat31::Zero()), r_(0.0), W_(obsInf)
{
    neighbourNodes_.push_back(node);
}


Factor1PosePoint2Plane::~Factor1PosePoint2Plane()
{
}


void Factor1PosePoint2Plane::evaluate_residuals()
{
    // r = <pi, Tp>
    Mat4 Tx = get_neighbour_nodes()->at(1)->get_state();
    SE3 T = SE3(Tx).inv();
    Tpoint_ = T.transform(z_point_);
    r_ = Mat1(z_plane_.head(3).dot(Tpoint_) + z_plane_(3));
}

void Factor1PosePoint2Plane::evaluate_jacobians()
{
    // it assumes you already have evaluated residuals
    Mat<3,6> Jr;
    Jr << -hat3(Tpoint_) , Mat3::Identity();
    J_ = z_plane_.head(3).transpose() * Jr;
}

void Factor1PosePoint2Plane::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor1PosePoint2Plane::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs point= \n" << z_point_
              << "obs plane =\n" << z_plane_
              << "\n Residuals= \n" << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Node ids: " << neighbourNodes_[0]->get_id()
              << std::endl;
}
