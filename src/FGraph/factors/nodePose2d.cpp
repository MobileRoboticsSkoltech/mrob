/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
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

NodePose2d::NodePose2d(const Mat31 &initial_x) : Node(3), x_(initial_x), linearization_x_(initial_x)
{
    assert(initial_x.rows() == 3 && "NodePose2d:: Incorrect dimension on initial state rows");
    assert(initial_x.cols() == 1 && "NodePose2d:: Incorrect dimension on initial state cols");
}

void NodePose2d::update(const Eigen::Ref<const MatX1> &dx)
{
    // TODO for incremental updates, this should be linearization_x + dx
    x_ += dx;
    x_(2) = wrap_angle(x_(2));

}

void NodePose2d::print() const
{
    std::cout << "Printing NodePose2d: " << id_
              << ", state = \n" << x_
              <<  "\nand neighbour factors " << neighbourFactors_.size()
              << std::endl;
}
