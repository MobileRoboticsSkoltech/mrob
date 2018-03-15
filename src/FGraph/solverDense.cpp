/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * DenseGaussNewton.cpp
 *
 *  Created on: Mar 3, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "solverDense.hpp"
#include <iostream>

using namespace fg;

DenseGaussNewton::DenseGaussNewton(std::shared_ptr<FGraph> fg):
        fg_(fg)
{
    fg->print();
    //TODO this does not work like this
    //lowerTriangularInformation_.selfadjointView<Eigen::Lower>();
}

DenseGaussNewton::~DenseGaussNewton()
{

}

void DenseGaussNewton::solveOnce()
{
    // iterate over all nodes
    auto iter = fg_->getBeginNodesIter();
    for ( ; iter != fg_->getEndNodesIter(); ++iter )
    {
        (*iter)->print();
        // Create the information matrix on Ax=b TODO como hacerlo sin romper abstract class??
        // Create the b vector from residuals

    }
    // Factorize I
    //
}

int DenseGaussNewton::solve()
{
    // Iteratively solve until convergence
    return 0;
}

/*MatX Factor2Poses3d::getJacobian(std::shared_ptr<Node> &n) const
{
    if(neighbourNodes_[0] == n)
        return J1_;
    if(neighbourNodes_[1] == n)
        return J2_;
    else
        // derivatives w.r.t other nodes are 0, regardless of the incorrectness of trying
        // get a Jacobian that does not define the factor
        return Mat6::Zero();
}*/
