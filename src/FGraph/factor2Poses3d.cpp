/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor2Poses3d.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "factor2Poses3d.hpp"

using namespace fg;


Factor2Poses3d::Factor2Poses3d(unsign_t id, const Mat61 &observation, const Mat6 &obsCov):
        Factor(id), dim_(6), obs_(observation), obsCov_(obsCov),
        J1_(Mat6::Zero()), J2_(Mat6::Zero())
{
}

Factor2Poses3d::~Factor2Poses3d()
{
}

void Factor2Poses3d::evaluate()
{

}
void Factor2Poses3d::evaluateLazy()
{

}
Mat6 Factor2Poses3d::getJacobian(unsign_t nodeId) const
{
    if(nodeId == id1_)
        return J1_;
    if(nodeId == id2_)
        return J2_;
    else
        // derivatives w.r.t other nodes are 0, regardless of the incorrectness of trying
        // get a jacobian that does not define the factor
        return Mat6::Zero();
}
