/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor1Pose3d.cpp
 *
 *  Created on: Mar 5, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "factor1Pose3d.hpp"
#include <iostream>

using namespace fg;


Factor1Pose3d::Factor1Pose3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsCov):
             Factor()
{
    neighbourNodes_.push_back(n1);
    obs_ = observation;//allocates memory and creates a copy
    dim_ = 6;
}

Factor1Pose3d::~Factor1Pose3d()
{
}

void Factor1Pose3d::evaluate()
{
    // Evaluate residual
    evaluateError();
    // Evaluate Jacobian
}

matData_t Factor1Pose3d::evaluateError()
{
    r_ = Mat61::Identity();
    return 0.0;
}
