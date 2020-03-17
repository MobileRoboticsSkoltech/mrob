/* Copyright 2018-2020 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * nodePoint3d.cpp
 *
 *  Created on: March 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/factors/nodePoint3d.hpp"

#include <iostream>
#include <assert.h>

using namespace mrob;

NodePoint3d::NodePoint3d(const Mat31 &initial_x) :
        Node(3), state_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 3 && "NodePoint3d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodePoint3d:: Incorrect dimension on initial state cols" );
}

NodePoint3d::~NodePoint3d()
{

}

void NodePoint3d::update(const Eigen::Ref<const MatX1> &dx)
{
    Mat31 dxf = dx;//XXX cast is necessary?
    state_ += dxf;
}

void NodePoint3d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    Mat31 dxf = dx;
    state_ = auxiliaryState_ + dxf;
}

void NodePoint3d::set_state(const Eigen::Ref<const MatX1> &x)
{
    state_ = x;
}

void NodePoint3d::set_auxiliary_state(const Eigen::Ref<const MatX1> &x)
{
    auxiliaryState_ = x;
}

const Eigen::Ref<const MatX> NodePoint3d::get_stateT() const
{
    assert(0 && "NodePoint3d::get_stateT(): this method should not be called.");
    return Mat3::Zero();
}

void NodePoint3d::print() const
{
    std::cout << "Printing NodePoint3d: " << id_
        << ", state = \n" << state_ << ",\n SE3 matrix: \n";
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
