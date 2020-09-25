/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 * factor_graph_solve_dense.cpp
 *
 *  Created on: Sep 22, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factor_graph_solve_dense.hpp"

using namespace mrob;

FGraphSolveDense::FGraphSolveDense()
{
    gradient_.resize(n);
    hessian_.resize(n,n);
}

FGraphSolveDense::~FGraphSolveDense()
{

}


