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

#include <unordered_map>
#include <deque>
#include "node.hpp"
#include "factor.hpp"

namespace skmr{
/**
 * This class provides the general structure for encoding Factor Graphs and
 * to support the implementation of the inference solution to the joint probability P(x,u,z).
 * The solution to this joint probability is equivalent to a Nonlinear Least Squares (NLS) problem.
 *
 * Factor Graphs are bipartite graphs, meaning that we express the relations from a set of vertices "nodes"
 * which include our state variables through a set of vertices "factors", capturing the inherent distribution
 * of the nodes variables due to observations.
 * Bipartite is in the sense that edges of the graph are always from nodes to factors or vice versa.
 *
 * We require two abstract classes,
 *  - Class Node: deque for fast iteration over uncertain sequence of elements.
 *  - Class Factor:  contained on a map whose key is its Id.
 *
 * Both data containers are stored in maps whose key is their Ids. By doing this, we can iterate and quickly find elements
 * in both data containers.
 *
 * Each problem instantaition should implement methods for solving the graph and storing the
 * necessary data
 */

class FGraph{
public:
    FGraph();
    virtual ~FGraph();
    virtual void solve() = 0;

    void addFactor(std::shared_ptr<Factor> &factor);
    void addNode(std::shared_ptr<Node> &node);
protected:
    // we use a deque to avoid memory reallocation on growing number of nodes.
    std::deque<std::shared_ptr<Node> > nodes_;
    std::unordered_map<unsigned int, std::shared_ptr<Factor> > factors_;//maybe an overkill?
};



}

#endif /* FGraph_HPP_ */
