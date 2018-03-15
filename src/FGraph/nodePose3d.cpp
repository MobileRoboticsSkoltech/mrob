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
#include <iostream>
#include <assert.h>

using namespace fg;

NodePose3d::NodePose3d(const Mat61 &initial_x) :
        Node(6), Tx_(initial_x)
{
    assert(initial_x.rows() == 6 && "NodePose3d:: Incorrect dimension on initial state rows" );
    assert(initial_x.cols() == 1 && "NodePose3d:: Incorrect dimension on initial state cols" );
    x_ = initial_x;// here x_ is allocated for the 1st time
}

NodePose3d::~NodePose3d()
{

}

void NodePose3d::update(const Eigen::Ref<const MatX1> &dx)
{
    Eigen::Ref<const Mat61> dxf(dx);
    // Tx and x are always sync, i.e., Tx = exp(x^)
    Tx_.update(dxf);
    x_ = Tx_.ln_vee();//this will cast to
}

void NodePose3d::print() const
{
    std::cout << "Printing Node Pose 3d, state = \n" <<
            x_ << "\nrepresenting the transformation\n" <<
            Tx_ << "\nand neighbour factors " <<
            neighbourFactors_.size() << std::endl;
}
