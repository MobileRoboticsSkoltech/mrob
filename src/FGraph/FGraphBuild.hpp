/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraphBuild.hpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SRC_FGRAPH_FGRAPHBUILD_HPP_
#define SRC_FGRAPH_FGRAPHBUILD_HPP_


#include "FGraph.hpp"

namespace fg{


/**
 * Class FGraphBuild creates all the required matrices for solving the LSQ problem.
 * Different options are provided:
 * 	- Adjacency matrix (plus indirect construction of Information)
 * 	- TODO Information matrix (direct)
 * 	- TODO Diagonal and information for Schur complement
 *
 * Later the chosen solver should be compliant with the
 * calculated matrices.
 */
class FGraphBuild : public FGraph
{
public:
    enum buildType{ADJACENCY = 0, ADJ2INFO, INFO, SCHUR};
    FGraphBuild(buildType type = ADJ2INFO, uint_t potNumberNodes = 512, uint_t potNumberFactors = 512);
    virtual ~FGraphBuild();
    void buildProblem();
    void solve();// XXX this goes here?

protected:
    /**
     * This protected method creates an Adjacency matrix, iterating over
     * all factors in the FG and including the squared root of the information
     * on every row W^(1/2)
     * The residuals are also calculated as b = W^(1/2)*r
     */
    void buildProblemAdjacency();
    /**
     * This method is a continuation from the previous, were
     * we build A and construct the Information by
     *     I = A'*W*A
     *  and the residuals are calculated as b = A'*W*r
     */
    void buildProblemAdj2Info();
    /**
     * TODO directly allocating components of the Information matrix
     */
    void buildProblemDirectInfo();


    buildType type_;
    SMatRow A_;
    SMatCol I_;
    MatX1 r_;


};


}


#endif /* SRC_FGRAPH_FGraphBuild_HPP_ */
