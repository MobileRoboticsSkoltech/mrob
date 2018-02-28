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

using namespace skmr;

NodePose3d::NodePose3d(int id, StateVect &initial_x) :
        Node(id), dim_(6), x_(initial_x)
{

}

NodePose3d::~NodePose3d()
{

}

void NodePose3d::update(StateVect &dx)
{
    //TODO this is not true for se(3)
    x_.noalias() += dx;
}
