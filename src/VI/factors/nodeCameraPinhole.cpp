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
 * nodeCameraPinhole.cpp
 *
 *  Created on: Aug 4, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/factors/nodeCameraPinhole.hpp"
#include <iostream>

using namespace mrob;

NodeCameraPinhole::NodeCameraPinhole(const Mat41 k_state, Node::nodeMode mode):
        Node(4,mode), k_state_(k_state), k_auxiliary_(k_state)
{

}

void NodeCameraPinhole::update(const Eigen::Ref<const MatX1> &dx)
{
    k_state_ += dx;
}

void NodeCameraPinhole::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    k_state_ = k_auxiliary_ + dx;
}

void NodeCameraPinhole::print() const
{
    std::cout << "Printing NodeCameraPinhole: " << id_
        << ", state [f_x, f_y, c_x, c_y] = \n" << k_state_ << std::endl;
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}


Mat21 mrob::CameraPinhole::project(const Mat41 &K, const Mat31 &P)
{
    // K = [fx, fy, cx, cy]
    Mat21 p;
    p << K[0] * P[0]/P[2] + K[2],
         K[1] * P[1]/P[2] + K[3];
    return p;
}
