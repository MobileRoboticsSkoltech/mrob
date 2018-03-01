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

namespace fg{
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
    FGraph(unsign_t potNumberNodes = 512, unsign_t potNumberFactors = 512);
    virtual ~FGraph();
    //virtual void solve() = 0;//TODO should we remove this?
    /**
     * Adds a factor, with already a unique id if returned true.
     * otherwise (false) it failed to add the new element, not unique key.
     * Note that the neighbbouring nodes of the factor should be already
     * specified when creating the node.
     */
    bool addFactor(std::shared_ptr<Factor> &factor);
    /**
      * Adds a node, with already a unique id if returned true.
      * otherwise (false) it failed to add the new element, not unique key
      * The neighbouring factors of the node should be already specified when
      * creating the node.
      */
    bool addNode(std::shared_ptr<Node> &node);
    /**
     * Connects a node and a factor by updating their internal list of neighbours
     * with the new connection. It updates both the node and the factor
     */
    void connectNodeFactor(std::shared_ptr<Node> &node, std::shared_ptr<Factor> &factor);
    /**
     * Disconnect node-factor and vice versa
     */
    void disconnectNodeFactor(std::shared_ptr<Node> &node, std::shared_ptr<Factor> &factor);
    unsign_t getFactorCount() const {return factorCount_;};
    unsign_t getNodeCount() const {return nodeCount_;};
protected:
    //XXX maybe unordered sets might work too... do we REALLY need Ids? Nodes Yes; Factors Maybe...
    std::unordered_map<unsign_t, std::shared_ptr<Node> >   nodes_;
    std::unordered_map<unsign_t, std::shared_ptr<Factor> > factors_;
    //std::unordered_set<std::shared_ptr<Factor> > factors_;
    unsign_t factorCount_, nodeCount_;
};



}

#endif /* FGraph_HPP_ */
