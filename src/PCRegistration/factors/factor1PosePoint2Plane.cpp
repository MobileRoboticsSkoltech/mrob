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


using namespace mrob;

Factor1PosePoint2Plane::Factor1PosePoint2Plane(const Mat31 &z_point, const Mat41 &z_plane,  std::shared_ptr<Node> &node,
        const Mat1 &obsInf):
    Factor(6,1), z_point_(z_point), z_plane_(z_plane), r_(0.0), W_(obsInf)
{
    r_ = Mat1(z_point.dot(z_plane.head(3)) + z_plane(3));//XXX why do we really want a mat1 instead of a double?
    neighbourNodes_.push_back(node);
}


Factor1PosePoint2Plane::~Factor1PosePoint2Plane()
{

}


void Factor1PosePoint2Plane::evaluate_residuals()
{

}

void Factor1PosePoint2Plane::evaluate_jacobians()
{

}

void Factor1PosePoint2Plane::evaluate_chi2()
{

}

void Factor1PosePoint2Plane::print() const
{

}
