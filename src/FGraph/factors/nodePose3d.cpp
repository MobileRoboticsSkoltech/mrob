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
 * nodePose3d.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/nodePose3d.hpp"

#include <iostream>
#include <cassert>

using namespace mrob;

NodePose3d::NodePose3d(const Mat4 &initial_x) :
        Node(6), state_(initial_x), auxiliaryState_(initial_x)
{
    // TODO remove me
    //assert(initial_x.rows() == 6 && "NodePose3d:: Incorrect dimension on initial state rows" );
    //assert(initial_x.cols() == 1 && "NodePose3d:: Incorrect dimension on initial state cols" );
    assert(isSE3(initial_x) && "NodePose3d:: Incorrect initial state, not an element of SE3" );
}

NodePose3d::NodePose3d(const SE3 &initial_x) :
		 Node(6), state_(initial_x), auxiliaryState_(initial_x)
{
	assert(isSE3(initial_x.T()) && "NodePose3d:: Incorrect initial state, not an element of SE3" );
}


void NodePose3d::update(VectRefConst &dx)
{
    Mat61 dxf = dx;

    // Tx and x are always sync, i.e., Tx = exp(x^)
    state_.update_lhs(dxf);
    // XXX regeneration of state is required, for now we do it every time. random? count?
    // XXX is it necessary?
    state_.regenerate();
}

void NodePose3d::update_from_auxiliary(VectRefConst &dx)
{
    Mat61 dxf = dx;
    state_ = auxiliaryState_;//we update from the auxiliary state
    state_.update_lhs(dxf);
}

void NodePose3d::set_state(MatRefConst &x)
{
	// casting is necessary for SE3 constructor, it does not handle a ref TODO
	Mat4 newState = x;
    state_ = SE3(newState);
}

void NodePose3d::set_auxiliary_state(MatRefConst &x)
{
	Mat4 newState = x;
    auxiliaryState_ = SE3(newState);
}

void NodePose3d::print() const
{
    std::cout << "Printing NodePose3d: " << id_
        << ", state = \n" << state_.ln_vee() << ",\n SE3 matrix: \n";
    state_.print();
}
