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
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/nodePose2d.hpp"
#include <iostream>

using namespace mrob;

NodePose2d::NodePose2d(const Mat31 &initial_x) : Node(3), state_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 3 && "NodePose2d:: Incorrect dimension on initial state rows");
    assert(initial_x.cols() == 1 && "NodePose2d:: Incorrect dimension on initial state cols");
}

void NodePose2d::update(const Eigen::Ref<const MatX1> &dx)
{
    state_ += dx;
    state_(2) = wrap_angle(state_(2));

}

void NodePose2d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    state_ = auxiliaryState_ + dx;
    state_(2) = wrap_angle(state_(2));

}


void NodePose2d::set_state(const Eigen::Ref<const MatX> &x)
{
    state_ = x;
    state_(2) = wrap_angle(state_(2));
}

void NodePose2d::set_auxiliary_state(const Eigen::Ref<const MatX> &x)
{
    auxiliaryState_ = x;
    auxiliaryState_(2) = wrap_angle(auxiliaryState_(2));
}

void NodePose2d::print() const
{
    std::cout << "Printing NodePose2d: " << id_
              << ", state = \n" << state_
              << std::endl;
}
