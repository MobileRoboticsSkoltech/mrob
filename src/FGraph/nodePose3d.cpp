/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * nodePose3d.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "nodePose3d.hpp"

using namespace fg;

NodePose3d::NodePose3d(int id, const Mat61 &initial_x) :
        Node(id), dim_(6), x_(initial_x), Tx_(initial_x)
{
}

NodePose3d::~NodePose3d()
{

}

void NodePose3d::update(const Mat61 &dx)
{
    // Tx and x are always sync, i.e., Tx = exp(x^)
    Tx_.update(dx);
    x_ = Tx_.ln_vee();
}
