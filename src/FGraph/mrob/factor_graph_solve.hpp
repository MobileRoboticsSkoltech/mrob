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

    void buildProblem();
    void solveOnce();
    void solveIncremental();

protected:
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and including the squared root of the information
     * on every row W^T/2 or W, depending on the solving method (QR or Chol)
     * The residuals are also calculated as b = W^(1/2)*r or b = A^T * W *r
     */
    void buildAdjacency();
    void buildAdjacency(SMat &A_new, SMat &W_new, MatX1 &r_new);

    /**
     * TODO directly allocating components of the Information matrix
     */
    void buildDirectInfo();
    void solveQR();
    void solveChol();

    void solveCholIncremental();
    void incrementalViaInformation(SMat &I_new, MatX1 &b_new);
    void incrementalViaL(SMat &I_new, MatX1 &b_new);

    void updateNodes(const MatX1 &dx_);

    // Variables for full solve
    solveType type_;

    SMatRow A_; //Adjacency matrix, as a Row sparse matrix
    SMatRow W_; //A block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_; // Residuals as given by the factors

    SMat I_; //Information matrix
    MatX1 b_; // Post-processed residuals, either A'*W*r for the normal equation or W*r for QR solvers

    // Variables for incremental solve
    long last_solved_node, last_solved_factor; // Index of last solved node and factor
    SMat L_; // Lower part of Cholesky decomposition of I_ matrix
};


}


#endif /* SRC_FACTOR_GRAPH_SOLVE_HPP_ */
