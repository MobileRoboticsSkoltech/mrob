/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 * nodeLandmark2d.cpp
 *
 *  Created on: Jul 27, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#include "mrob/factors/nodeLandmark2d.hpp"

#include <iostream>
#include <assert.h>


using namespace mrob;

NodeLandmark2d::NodeLandmark2d(const Mat21 &initial_x) :
        Node(2), state_(initial_x), auxiliaryState_(initial_x)
{
    assert(initial_x.rows() == 2 && "NodeLandmark2d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodeLandmark2d:: Incorrect dimension on initial state cols" );
}

NodeLandmark2d::~NodeLandmark2d()
{

}

void NodeLandmark2d::update(const Eigen::Ref<const MatX1> &dx)
{
    //TODO test cast
    Mat21 dxf = dx;
    state_ += dxf;
}

void NodeLandmark2d::update_from_auxiliary(const Eigen::Ref<const MatX1> &dx)
{
    //TODO test cast
    Mat21 dxf = dx;
    state_ = auxiliaryState_ + dxf;
}

void NodeLandmark2d::set_state(const Eigen::Ref<const MatX> &x)
{
    // cast is done by Eigen
    state_ = x;
}

void NodeLandmark2d::set_auxiliary_state(const Eigen::Ref<const MatX> &x)
{
    auxiliaryState_ = x;
}

void NodeLandmark2d::print() const
{
    std::cout << "Printing NodeLandmark2d: " << id_
        << ", state = \n" << state_;
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
