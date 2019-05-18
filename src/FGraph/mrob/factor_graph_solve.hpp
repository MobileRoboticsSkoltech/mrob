/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
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
 * Later the chosen solver should be compliant with the
 * calculated matrices.
 */
class FGraphSolve: public FGraph
{
public:
    enum solveMethod{CHOL=0, SCHUR};

    FGraphSolve(solveMethod method = CHOL, uint_t potNumberNodes = 512, uint_t potNumberFactors = 512);
    virtual ~FGraphSolve();
    /**
     * Solves the batch problem, by linearizing, ordering
     */
    void solve_batch();
    /**
     * Solve incremental, uses previous solution and linearization point
     * to incrementally solve the problem
     */
    void solve_incremental();

    /**
     * Evaluates the current solution chi2.
     *
     * Variable relinearizeProblemFlag:
     *      - (default) true: Recalculates residuals.
     *      - false: Uses the previous calculated residuals
     */
    matData_t chi2(bool evaluateResidualsFlag = true);

    std::vector<MatX1> get_estimated_state();
    //TODO are this necessary?
    void set_solve_method(solveMethod method) {method_ = method;};
    solveMethod get_solve_method() { return method_;};


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
    void build_adjacency_incremental(SMatCol &A_new, SMatCol &W_new, MatX1 &r_new);
    void build_direct_info();//TODO

    /**
     * Solve the systems using Cholesky decomposition.
     * In addition, it creates the information matrix as
     *              I = A^T * W * A
     * The residuals are also calculated as b = A^T * W *r
     */
    void solve_cholesky();
    void solve_chol_incremental();

    /**
     * Auxiliary function that updates all nodes with the current solution,
     * this should be called after solving the problem
     */
    void update_nodes();

    // Variables for solving the FG
    solveMethod method_;

    /**
     * Minimum degree ordering data structures.
     * permutation: from the node vector index to the ordered index
     * permutationInverse_;  back from the ordered index to the node vector
     */
    std::vector<id_t> permutation_, permutationInverse_;
    SMatRow A_; //Adjacency matrix, as a Row sparse matrix
    SMatRow W_; //A block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_; // Residuals as given by the factors

    SMatCol I_; //Information matrix
    MatX1 b_; // Post-processed residuals, A'*W*r

    // Variables for incremental solve
    //TODO remove long data type, and check they are necessary
    long last_stateDim, last_obsDim; // stateDim and obsDim of the last solve
    long last_solved_node, last_solved_factor; // Index of last solved node and factor
    SMatCol L00, L10, L11, I11; // Lower part of Cholesky decomposition of I_ matrix
    MatX1 y_; // Solution of Ly = b

    // Correction deltas DEPRECATED?
    MatX1 dx_;
    //TODO ordering matrix for variables/nodes
};


}


#endif /* SRC_FACTOR_GRAPH_SOLVE_HPP_ */
