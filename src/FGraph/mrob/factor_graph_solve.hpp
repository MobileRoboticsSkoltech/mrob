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
 * factor_graph_solve.hpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SRC_FACTOR_GRAPH_SOLVE_HPP_
#define SRC_FACTOR_GRAPH_SOLVE_HPP_


#include "mrob/factor_graph.hpp"
#include "mrob/time_profiling.hpp"

namespace mrob {


/**
 * Class FGraphSolve creates all the required matrices for solving the LSQ problem.
 * Different options are provided:
 * 	- Adjacency matrix (plus indirect construction of Information)
 * 	- TODO Information matrix (direct)
 * 	- TODO Diagonal and information for Schur complement
 *
 * Routines provide different optimization methods:
 *  - Gauss-Newton (GN) using Cholesky LDLT with minimum degree ordering
 *  - Levenberg–Marquardt (LM) (Nocedal Ch.10) using spherical
 *                     trust region alg. (Nocedal 4.1) to estimate a "good" lambda.
 *                     Bertsekas p.105 proposes a similar heuristic approach for the trust
 *                     region, which we convert to lambda estimation (we follow Bertsekas' notation in code).
 *  - TODO Gradient 1st order based with preconditioning
 *  - TODO Dogleg (DL) (Nocedal 4.3)
 *  - TODO Conjugate gradient method (Nocedal 7.2)
 */
class FGraphSolve: public FGraph
{
public:
    /**
     * This enums all matrix building methods available
     */
    enum matrixMethod{ADJ=0, SCHUR};
    /**
     * This enums optimization methods available:
     *  - Gauss Newton
     *  - Levenberg Marquardt (trust-region-like for lambda adjustment)
     */
    enum optimMethod{GN=0, LM};

    FGraphSolve(matrixMethod method = ADJ, optimMethod optim = GN);
    virtual ~FGraphSolve();

    /**
     * Solve call the corresponding routine on the class parameters or
     * ultimately on the function input,
     * by default optim method is Gauss Newton
     */
    void solve(optimMethod method = GN, uint_t maxIters = 20);
    /**
     * Evaluates the current solution chi2.
     *
     * Variable relinearizeProblemFlag:
     *      - (default) true: Recalculates residuals.
     *      - false: Uses the previous calculated residuals
     */
    matData_t chi2(bool evaluateResidualsFlag = true);
    /**
     * Returns a Reference to the solution vector
     * of all variables, vectors, matrices, etc.
     */
    std::vector<MatX> get_estimated_state();

    /**
     * Functions to set the matrix method building
     */
    void set_build_matrix_method(matrixMethod method) {matrixMethod_ = method;};
    matrixMethod get_build_matrix_method() { return matrixMethod_;};


protected:
    /**
     * build problem creates an information matrix L, W and a vector b
     *
     * It chooses from building the information from the adjacency matrix,
     * directly building info or schur (TODO)
     *
     * If bool useLambda is true, it also stores a vector D2 containing the diagonal
     * of the information matrix L
     */
    void build_problem(bool useLambda = false);
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and creates a block diagonal matrix W with each factors information.
     * As a result, residuals, Jacobians and chi2 values are up to date
     *
     */
    void build_adjacency();
    /**
     * From the adjacency matrix it creates the information matrix as
     *              L = A^T * W * A
     * The residuals are also calculated as b = A^T * W *r
     */
    void build_info_adjacency();
    void build_schur(); // TODO

    /**
     * Once the matrix L is generated, it solves the linearized LSQ
     * by using the Gauss-Newton algorithm
     *
     * Input useLambda (default false) builds the GN problem with lambda factor on the diagonal
     *    L = A'*A + lambda * I
     */
    void optimize_gauss_newton(bool useLambda = false);

    /**
     * It generates the information matrix as
     *              L' = L +  lambda * I
     *
     * TODO, the preconditioning could be adapted to expected values, such as w < pi and v < avg
     *               L' = L +  lambda * D2
     *
     * Iteratively updates the solution given the right estimation of lambda
     * Parameters are necessary to be specified in advance, o.w. it would use default values.
     *
     * input maxIters, before returning a result
     *
     * output: number of iterations it took to converge.
     *    0 when incorrect solution
     */
    uint_t optimize_levenberg_marquardt(uint_t maxIters);

    /**
     * Function that updates all nodes with the current solution,
     * this must be called after solving the problem
     */
    void update_nodes();

    /**
     * Synchronize state variable in all nodes
     * exactly value the current state.
     *
     * Usually this function un-does an incorrect update of the state.
     */
    void synchronize_nodes_state();

    /**
     * Synchronize auxiliary state variables in all nodes
     * exactly value the current state.
     * This function is used when we will update a solution
     * but it needs verification, so we book-keep at auxiliary.
     */
    void synchronize_nodes_auxiliary_state();

    // Variables for solving the FGraph
    matrixMethod matrixMethod_;
    optimMethod optimMethod_;

    uint_t N_; // total number of state variables
    uint_t M_; // total number of observation variables

    SMatRow A_; //Adjacency matrix, as a Row sparse matrix
    SMatRow W_; //A block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_; // Residuals as given by the factors

    SMatCol L_; //Information matrix
    MatX1 b_; // Post-processed residuals, A'*W*r

    // Correction deltas
    MatX1 dx_;

    // Particular parameters for Levenberg-Marquard
    matData_t lambda_; // current value of lambda
    matData_t solutionTolerance_;
    MatX1 diagL_; //diagonal matrix (vector) of L to update it efficiently

    // time profiling
    TimeProfiling time_profiles_;
};


}


#endif /* SRC_FACTOR_GRAPH_SOLVE_HPP_ */
