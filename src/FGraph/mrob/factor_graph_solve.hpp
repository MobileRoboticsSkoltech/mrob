/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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


namespace mrob {


/**
 * Class FGraphSolve creates all the required matrices for solving the LSQ problem.
 * Different options are provided:
 * 	- Adjacency matrix (plus indirect construction of Information)
 * 	- TODO Information matrix (direct)
 * 	- TODO Diagonal and information for Schur complement
 *
 * Routines provide different optiomization methods:
 *  - optimize_gn (Gauss-Newton) using Cholesky LDLT with minimum degree ordering
 *  - optimize_lm (Levenberg–Marquardt) TODO
 */
class FGraphSolve: public FGraph
{
public:
    /**
     * This enums all matrix building methods available
     */
    enum matrixMethod{ADJ=0, SCHUR};
    /**
     * This enums optimization methods available
     */
    enum optimizationMethod{GAUSS_NEWTON=0, LEVENBERG_MARQUARDT};

    FGraphSolve(matrixMethod method = ADJ, optimizationMethod = GAUSS_NEWTON, uint_t potNumberNodes = 512, uint_t potNumberFactors = 512);
    virtual ~FGraphSolve();
    /**
     * Solves the batch problem, by linearizing, ordering
     */
    void solve_batch();// TODO deprecated


    void solve();
    /**
     * Once the matrix is generated, it solve the linearized LSQ
     * by using the Gauss-Newton algorithm
     */
    void optimize_gauss_newton();

    /**
     * Once the matrix is generated, it solve the linearized LSQ
     * by using the Levenberg-Marquardt algorithm
     */
    void optimize_levenberg_marquardt();

    /**
     * Evaluates the current solution chi2.
     *
     * Variable relinearizeProblemFlag:
     *      - (default) true: Recalculates residuals.
     *      - false: Uses the previous calculated residuals
     */
    matData_t chi2(bool evaluateResidualsFlag = true);

    std::vector<MatX1> get_estimated_state();

    void set_matrix_method(matrixMethod method) {matrixMethod_ = method;};
    matrixMethod get_matrix_method() { return matrixMethod_;};

    /**
     * Particular parameters for Levenberg-Marquard
     * TODO
     */


protected:
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and creates a block diagonal matrix W with each factors information.
     *
     * During build adjacency also we keep track of the number of factors for each
     * node and order them according to the minimum order degree. The permutation vector
     * is stored on the class variable permutation_
     *
     */
    void build_adjacency();
    void build_info_adjacency();

    /**
     * Solve the systems using Cholesky LDLT decomposition with AMD ordering
     * In addition, it creates the information matrix as
     *              I = A^T * W * A
     * The residuals are also calculated as b = A^T * W *r
     *
     * Note: LLT provides similar results
     */
    void solve_cholesky();

    /**
     * Auxiliary function that updates all nodes with the current solution,
     * this must be called after solving the problem
     */
    void update_nodes();

    // Variables for solving the FGraph
    matrixMethod matrixMethod_;
    optimizationMethod optimMethod_;

    uint_t N_; // total number of state variables
    uint_t M_; // total number of observation variables

    SMatRow A_; //Adjacency matrix, as a Row sparse matrix
    SMatRow W_; //A block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_; // Residuals as given by the factors

    SMatCol I_; //Information matrix
    MatX1 b_; // Post-processed residuals, A'*W*r

    // Correction deltas
    MatX1 dx_;

    // Execution evaluation TODO, do a pair of time, strings and plot things more properly
    std::vector<double> time_profiles_;//used for time profiling functions

    // Particular parameters for Levenberg-Marquard
    double lambda_;
};


}


#endif /* SRC_FACTOR_GRAPH_SOLVE_HPP_ */
