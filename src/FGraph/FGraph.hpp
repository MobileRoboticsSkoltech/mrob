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
 *  - Class Node
 *  - Class Factor
 *
 * Both data containers are stored in unordered maps whose key is their Ids. By doing this, we can
 * iterate and quickly find elements in both data containers.
 *
 * Each problem instantaition should implement methods for solving the graph and storing the
 * necessary data, such as information matrix, factorizations, etc.
 */

class FGraph{
public:
    FGraph(unsigned int potNumberNodes = 512, unsigned int potNumberFactors = 512);
    virtual ~FGraph();
    //virtual void solve() = 0;//TODO should we remove this?
    /**
     * Adds a factor, with already a unique id if returned true.
     * otherwise (false) it failed to add the new element, not unique key
     */
    bool addFactor(std::shared_ptr<Factor> &factor);
    /**
      * Adds a noder, with already a unique id if returned true.
      * otherwise (false) it failed to add the new element, not unique key
      */
    bool addNode(std::shared_ptr<Node> &node);
    unsigned int getFactorCount() const {return factorCount_;};
    unsigned int getNodeCount() const {return nodeCount_;};
protected:
    //XXX maybe unordered sets might work too... do we REALLY need Ids?
    std::unordered_map<unsigned int, std::shared_ptr<Node> >   nodes_;
    std::unordered_map<unsigned int, std::shared_ptr<Factor> > factors_;
    unsigned int factorCount_, nodeCount_;
};



}

#endif /* FGraph_HPP_ */
