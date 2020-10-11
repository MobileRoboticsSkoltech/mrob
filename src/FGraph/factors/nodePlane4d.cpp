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
 * nodePlane4d.cpp
 *
 *  Created on: Oct 10, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/factors/nodePlane4d.hpp"
#include <iostream>
#include <assert.h>

using namespace mrob;

NodePlane4d::NodePlane4d(const Mat41 &initial_x):
    Node(4), state_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 4 && "NodePlane4d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodePlane4d:: Incorrect dimension on initial state cols" );
    // ensure that plane 4d \in P^3
    state_.normalize();
    auxiliaryState_.normalize();
}

NodePlane4d::~NodePlane4d()
{

}

void NodePlane4d::update(const Eigen::Ref<const MatX1> &dx)
{
    Mat41 dpi = dx;
    state_ += dx;
    state_.head(3).normalize();
}

void NodePlane4d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    Mat41 dpi = dx;
    state_ = auxiliaryState_ + dx;
    state_.head(3).normalize();
}

void NodePlane4d::print() const
{
    std::cout << "Printing NodePlane4d: " << id_
        << ", state = \n" << state_ << ",\n";
    std::cout  <<  "and neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
