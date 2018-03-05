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
    std::cout << "We are here" << std::endl;
    fg->print();
}

DenseGaussNewton::~DenseGaussNewton()
{

}

void DenseGaussNewton::buildProblem()
{

}
