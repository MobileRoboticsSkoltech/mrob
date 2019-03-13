//
// Created by Konstantin on 14/01/2019.
//
#include "mrob/factors/nodePose2d.hpp"
#include <iostream>

using namespace mrob;

NodePose2d::NodePose2d(const Mat31 &initial_x) : Node(3), x_(initial_x) {
    assert(initial_x.rows() == 3 && "NodePose2d:: Incorrect dimension on initial state rows");
    assert(initial_x.cols() == 1 && "NodePose2d:: Incorrect dimension on initial state cols");
}

void NodePose2d::update(const Eigen::Ref<const MatX1> &dx) {
    x_ += dx;
}

void NodePose2d::print() const
{
    std::cout << "Printing NodePose2d: " << id_
              << ", state = \n" << x_
              <<  "\nand neighbour factors " << neighbourFactors_.size()
              << std::endl;
}
