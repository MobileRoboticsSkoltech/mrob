/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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


void NodePose2d::set_state(const Eigen::Ref<const MatX1> &x)
{
    state_ = x;
    state_(2) = wrap_angle(state_(2));
}

void NodePose2d::set_auxiliary_state(const Eigen::Ref<const MatX1> &x)
{
    auxiliaryState_ = x;
    auxiliaryState_(2) = wrap_angle(auxiliaryState_(2));
}

void NodePose2d::print() const
{
    std::cout << "Printing NodePose2d: " << id_
              << ", state = \n" << state_
              <<  "\nand neighbour factors " << neighbourFactors_.size()
              << std::endl;
}
