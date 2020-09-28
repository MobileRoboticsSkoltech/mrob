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

class EFSolve: public FGraphSolve
{
public:
    EFSolve();
    ~EFSolve();

    /**
     * Solve the alignment problem for a stream of observations
     * from pose x_origin = I to x_f
     * w.r.t the final pose
     */
    void solve_1_pose();
    /**
     * Solve the full problem involving multiple poses
     */
    void solve_planes();
};

}//end namespace

#endif /* FACTOR_GRAPH_EF_HPP_ */
