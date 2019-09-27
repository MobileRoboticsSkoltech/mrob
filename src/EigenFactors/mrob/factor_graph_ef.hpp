/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * factor_graph_ef.hpp
 *
 *  Created on: Sep 27, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTOR_GRAPH_EF_HPP_
#define FACTOR_GRAPH_EF_HPP_

#include "mrob/factor_graph_solve.hpp"

/**
 * This class, inherits from solve_factor_graph and aims to provide the
 * additional structure required to process Eigen Factors as plane factors
 *
 * TODO: merge this in solve
 */


namespace mrob {

class EfFGraphSolve: public FGraphSolve
{
public:
    EfFGraphSolve();
    ~EfFGraphSolve;
};

}//end namespace

#endif /* FACTOR_GRAPH_EF_HPP_ */
