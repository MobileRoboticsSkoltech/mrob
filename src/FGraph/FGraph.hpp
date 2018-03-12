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

#include <unordered_set>
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
 * Both data containers are stored in unordered sets whose key is their adresses. By doing this, we can
 * iterate and quickly find elements in both data containers.
 *
 * Each problem instantaition should implement methods for solving the graph and storing the
 * necessary data, such as information matrix, factorizations, etc.
 */

class FGraph{
public:
    FGraph(uint_t potNumberNodes = 512, uint_t potNumberFactors = 512);
    virtual ~FGraph();
    /**
     * Adds a factor, if it is not already on the set.
     * Note that the connecting nodes of the factor should be already
     * specified when creating the factor.
     *
     * This function includes the factor into its connected
     * nodes.
     *
     * Modifications of the structure of the graph are allowed
     * by removing the factor and adding the new updated one.
     */
    bool addFactor(std::shared_ptr<Factor> &factor);
    /**
      * Adds a node if it was not already on the set.
      */
    bool addNode(std::shared_ptr<Node> &node);
    /**
     * Removes a Factor and on all the connected Nodes
     * list of factors, it is removed as well
     */
    void rmFactor(std::shared_ptr<Factor> &factor);
    /**
     * Removes Node from list. TODO should we eliminate
     * all factors pointing to that node?
     */
    void rmNode(std::shared_ptr<Node> &node);
    void print(bool complete = false) const;
    /**
     * get the iterator for the Nodes set
     */
    std::unordered_set<std::shared_ptr<Node> >::iterator
    getBeginNodesIter(){return nodes_.begin();};
    /**
     *
     */
    std::unordered_set<std::shared_ptr<Node> >::iterator
    getEndNodesIter() {return nodes_.end();};

    //TODO
    void saveGraph() const;
    void loadGraph();
protected:
    std::unordered_set<std::shared_ptr<Node> >   nodes_;
    std::unordered_set<std::shared_ptr<Factor> > factors_;
};



}

#endif /* FGraph_HPP_ */
