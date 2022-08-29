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
 * nodeLandmark3d.cpp
 *
 *  Created on: March 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/nodeLandmark3d.hpp"

#include <iostream>
#include <cassert>

using namespace mrob;

NodeLandmark3d::NodeLandmark3d(const Mat31 &initial_x, Node::nodeMode mode) :
    Node(3, mode), state_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 3 && "NodeLandmark3d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodeLandmark3d:: Incorrect dimension on initial state cols" );
}


void NodeLandmark3d::update(const Eigen::Ref<const MatX1> &dx)
{
    Mat31 dxf = dx;//XXX cast is necessary?
    state_ += dxf;
}

void NodeLandmark3d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    Mat31 dxf = dx;
    state_ = auxiliaryState_ + dxf;
}

void NodeLandmark3d::set_state(const Eigen::Ref<const MatX> &x)
{
	// cast is done by Eigen
    state_ = x;
}

void NodeLandmark3d::set_auxiliary_state(const Eigen::Ref<const MatX> &x)
{
    auxiliaryState_ = x;
}

void NodeLandmark3d::print() const
{
    std::cout << "Printing NodeLandmark3d: " << id_
        << ", state = \n" << state_;
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
