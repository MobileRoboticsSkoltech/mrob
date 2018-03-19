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
    //lowerTriangularInformation_.selfadjointView<Eigen::Lower>();
    // Create the matrices on Ax=b
    uint_t state_dim = 3;//read from fg?
    MatX A_(state_dim, state_dim);
    MatX1 b_(state_dim);
}

DenseGaussNewton::~DenseGaussNewton()
{

}

void DenseGaussNewton::solveOnce()
{


    // 1) Iterate over all nodes, for now, we don't elect a subset of them
    auto iter = fg_->getBeginNodesIter();
    for ( ; iter != fg_->getEndNodesIter(); ++iter )
    {
        // 2) Every Node has a vector of neighbour factors, we iterate over them to calculate
        // residuals and the Jacobians
        auto neighFactors = (*iter)->getNeighbourFactors();
        for (uint_t i = 0; i < neighFactors->size();  ++i)
        {
            std::shared_ptr<fg::Factor> factor = (*neighFactors)[i];//shared pointer to a base class factor
            factor->evaluate();


        }

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
