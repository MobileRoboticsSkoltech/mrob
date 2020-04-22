/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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
#include <assert.h>

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

NodePose3d::~NodePose3d()
{

}

void NodePose3d::update(const Eigen::Ref<const MatX1> &dx)
{
    Mat61 dxf = dx;

    // Tx and x are always sync, i.e., Tx = exp(x^)
    state_.update_lhs(dxf);
    // XXX regeneration of state is required, for now we do it every time. random? count?
    // XXX is it necessary?
    state_.regenerate();
}

void NodePose3d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    Mat61 dxf = dx;
    state_ = auxiliaryState_;//we update from the auxiliary state
    state_.update_lhs(dxf);
}

void NodePose3d::set_state(const Eigen::Ref<const MatX> &x)
{
	// casting is necessary for SE3 constructor, it does not handle a ref TODO
	Mat4 newState = x;
    state_ = SE3(newState);
}

void NodePose3d::set_auxiliary_state(const Eigen::Ref<const MatX> &x)
{
	Mat4 newState = x;
    auxiliaryState_ = SE3(newState);
}

void NodePose3d::print() const
{
    std::cout << "Printing NodePose3d: " << id_
        << ", state = \n" << state_.ln_vee() << ",\n SE3 matrix: \n";
    state_.print();
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
