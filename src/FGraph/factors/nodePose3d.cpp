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

#include "mrob/factors/nodePose3d.hpp"

#include <iostream>
#include <assert.h>

using namespace mrob;

NodePose3d::NodePose3d(const Mat61 &initial_x) :
        Node(6), x_(initial_x),Tx_(initial_x), linearization_x_(initial_x)
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

    // XXX debuging, when upodates are too large, clip them to PI/4
    double dw = dxf.head(3).norm();
    if (dw > M_PI/4 )
    {
        std::cout << "incorrect update at node "<< this->id_ << "update = " << dxf <<  std::endl;
        //dxf *= M_PI/4.0/dw;


        this->print();
        auto list = this->get_neighbour_factors();
        for ( auto f : *list )
        {
            f->print(); //->print();

        }

    }
    // Tx and x are always sync, i.e., Tx = exp(x^)
    Tx_.update_lhs(dxf);
    x_ = Tx_.ln_vee();//this will cast to
}

void NodePose3d::set_state(const Eigen::Ref<const MatX1> &x)
{
    x_ = x;
    Tx_ = SE3(x_);
}

const Eigen::Ref<const MatX> NodePose3d::get_stateT() const
{
    return Tx_.T();
}

void NodePose3d::print() const
{
    std::cout << "Printing NodePose3d: " << id_
        << ", state = \n" << x_ << ",\n SE3 matrix: \n";
    Tx_.print();
    std::cout  <<  "\nand neighbour factors " << neighbourFactors_.size()
        << std::endl;
}
