/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraphSolve.hpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SRC_FGRAPH_FGRAPHSOLVE_HPP_
#define SRC_FGRAPH_FGRAPHSOLVE_HPP_


#include "FGraph.hpp"

namespace fg{


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
    void solveIncremental();//TODO

protected:
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and including the squared root of the information
     * on every row W^T/2 or W, depending on the solving method (QR or Chol)
     * The residuals are also calculated as b = W^(1/2)*r or b = A^T * W *r
     */
    void buildAdjacency();
    /**
     * TODO directly allocating components of the Information matrix
     */
    void buildDirectInfo();
    void solveQR();
    void solveChol();


    solveType type_;
    SMatRow A_;//Adjacency matrix, as a Row sparse matrix
    SMat I_;//Information matrix
    SMatRow W_;//a block diagonal information matrix. For types Adjacency it calculates its block transposed squared root
    MatX1 r_;// residuals as given by the factors
    MatX1 b_;// postprocessed residuals, either A'*W*r for the normal equation or W*r for QR solvers


};


}


#endif /* SRC_FGRAPH_FGraphSolve_HPP_ */
