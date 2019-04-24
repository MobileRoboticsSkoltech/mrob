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
    enum solveType{QR = 0, CHOL_ADJ, CHOL, SCHUR};

    FGraphSolve(solveType type = CHOL_ADJ, uint_t potNumberNodes = 512, uint_t potNumberFactors = 512);
    virtual ~FGraphSolve();
    void solve_once();
    void solve_incremental();

    /**
     * This function is called after solving the problem
     */
    double evaluate_chi2() {return r_.dot(r_);};

    std::vector<MatX1> get_estimated_positions();
    std::shared_ptr<Node>& get_node(uint_t pos);

protected:
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and including the squared root of the information
     * on every row W^T/2 or W, depending on the solving method (QR or Chol)
     * The residuals are also calculated as b = W^(1/2)*r or b = A^T * W *r
     */
    void build_adjacency();
    void build_adjacency_incremental(SMatCol &A_new, SMatCol &W_new, MatX1 &r_new);
    void build_problem();

    /**
     * TODO directly allocating components of the Information matrix
     */
    void build_direct_info();
    void solve_QR();
    void solve_cholesky();
    void solve_chol_incremental();

    /**
     * Auxiliary function that updates all nodes with the current solution,
     * this should be called after solving the problem
     */
    void update_nodes();

    // Variables for full solve
    solveType type_;

    SMatRow A_; //Adjacency matrix, as a Row sparse matrix
    SMatRow W_; //A block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_; // Residuals as given by the factors

    SMatCol I_; //Information matrix
    MatX1 b_; // Post-processed residuals, either A'*W*r for the normal equation or W*r for QR solvers

    // Variables for incremental solve TODO remove long data type
    long last_stateDim, last_obsDim; // stateDim and obsDim of the last solve
    long last_solved_node, last_solved_factor; // Index of last solved node and factor
    SMatCol L00, L10, L11, I11; // Lower part of Cholesky decomposition of I_ matrix
    MatX1 y_; // Solution of Ly = b

    // Correction deltas
    MatX1 dx_;
};


}


#endif /* SRC_FACTOR_GRAPH_SOLVE_HPP_ */
