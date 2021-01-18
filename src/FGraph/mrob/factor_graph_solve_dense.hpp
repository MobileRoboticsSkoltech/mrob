/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * factor_graph_solve_dense.hpp
 *
 *  Created on: Sep 3, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTOR_GRAPH_SOLVE_DENSE_HPP_
#define FACTOR_GRAPH_SOLVE_DENSE_HPP_

#include "mrob/optimizer.hpp"
#include "mrob/factor_graph.hpp"
#include "mrob/time_profiling.hpp"

namespace mrob {


/**
 * Class FGraphSolveDense solve a factor graph problem assuming dense
 * precision matrix.
 *
 * It inherits from two classes:
 *  - FGraph: structure for adding generic factors
 *  - Optimizer: Optimization methods given some abstract routines
 */
class FGraphSolveDense: public FGraph, public OptimizerDense
{
  public:
    FGraphSolveDense();
    ~FGraphSolveDense();

    // Function from the parent class Optimizer
    virtual matData_t calculate_error() override;
    virtual void calculate_gradient_hessian() override;
    virtual void update_state(const MatX1 &dx) override;
    virtual void bookkeep_state() override;
    virtual void update_state_from_bookkeep() override;

};


}//namespace

#endif /* FACTOR_GRAPH_SOLVE_DENSE_HPP_ */
