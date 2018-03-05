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
    // get list of nodes? fg_->
    //for ()
    {
        // Create the information matrix on Ax=b
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
