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

NodePose3d::NodePose3d(const Mat61 &initial_x) :
        Node(6), state_(initial_x),stateT_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 6 && "NodePose3d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodePose3d:: Incorrect dimension on initial state cols" );
}

NodePose3d::~NodePose3d()
{

}

void NodePose3d::update(const Eigen::Ref<const MatX1> &dx)
{
    Mat61 dxf = dx;

    // Tx and x are always sync, i.e., Tx = exp(x^)
    stateT_.update_lhs(dxf);
    state_ = stateT_.ln_vee();//this will cast to
    stateT_ = SE3(state_);//XXX is it necessary to update this every update? random? or count?
}

void NodePose3d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    Mat61 dxf = dx;
    SE3 T(auxiliaryState_);
    T.update_lhs(dxf);
    state_ = T.ln_vee();
    stateT_ = T;
}

void NodePose3d::set_state(const Eigen::Ref<const MatX1> &x)
{
    state_ = x;
    stateT_ = SE3(state_);
}

void NodePose3d::set_auxiliary_state(const Eigen::Ref<const MatX1> &x)
{
    auxiliaryState_ = x;
}

const Eigen::Ref<const MatX> NodePose3d::get_stateT() const
{
    return stateT_.T();
}

void NodePose3d::print() const
{
    std::cout << "Printing NodePose3d: " << id_
        << ", state = \n" << state_ << ",\n SE3 matrix: \n";
    stateT_.print();
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
