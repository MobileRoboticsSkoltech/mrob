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
	enum buildType{Adjacency = 0, Adj2Info, Info};
    FGraphBuild();
    virtual ~FGraphBuild();
    void buildproblem();
    void solve();
  protected:
    buildType type_;
    MatRow A_;
    MatCol I_;


};


}


#endif /* SRC_FGRAPH_FGraphBuild_HPP_ */
