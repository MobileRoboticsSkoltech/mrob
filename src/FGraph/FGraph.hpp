/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraph.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FGRAPH_HPP_
#define FGRAPH_HPP_

#include "node.hpp"
#include "factor.hpp"

namespace skmr{
/**
 * This class provides the general structure for encoding Factor Graphs and
 * implementing the inference solution to the joint probability P(x,u,z).
 * The solution to this joint probability is equivalent to a Nonlinear Least Squares (NLS) problem.
 *
 * Factor Graphs are bipartite graphs, meaning that we express the relations from a set of vertices "nodes"
 * which include our state variables through a set of vertices "factors", capturing the inherent distribution
 * of the nodes variables due to observations.
 * Bipartite is in the sense that edges of the graph are always from nodes to factors or vice versa.
 *
 * We require two abstract classes for doing that,
 *  - Class Node
 *  - Class Factor
 */

class FGraph{
public:
    FGraph();
    ~FGraph();
protected:
    //std::vector<skmr::Node*> nodes_;
};



}

#endif /* FGraph_HPP_ */
