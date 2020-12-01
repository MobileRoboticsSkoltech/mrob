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
 * nodeImuBias.cpp
 *
 *  Created on: Dec 1, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/nodeImuBias.hpp"
#include "mrob/factors/nodePose3d.hpp"
#include <iostream>

using namespace mrob;

NodeImuBias::NodeImuBias(const Mat61 &initial_x):
        Node(6), state_(initial_x), auxiliary_state_(initial_x)
{

}

NodeImuBias::~NodeImuBias()
{

}

void NodeImuBias::update(const Eigen::Ref<const MatX1> &dx)
{
    //TODO check that update of bias is linear and Sign
    state_ += dx;
}

void NodeImuBias::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    state_ = auxiliary_state_ + dx;
}
void NodeImuBias::print() const
{
    //TODO see examples in Fgraph: factors nodePose3d.cpp
    std::cout << "Program me" << std::endl;
}

