/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor_graph.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_GRAPH_HPP_
#define FACTOR_GRAPH_HPP_

//#include <unordered_set>
//#include <deque>//TODO change for long allocations
#include <vector>
#include "skmr/node.hpp"
#include "skmr/factor.hpp"

namespace skmr{
/**
 * This class provides the general structure for encoding Factor Graphs and
 * to support the implementation of the inference solution to the joint probability P(x,u,z).
 * The solution to this joint probability is equivalent to a Nonlinear Least Squares (NLSQ) problem.
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
 * Both data containers are stored in unordered sets whose keys are their adresses. By doing this, we can
 * iterate and quickly find elements in both data containers.
 *
 * Each problem instantaition should implement methods for solving the graph and storing the
 * necessary data, such as information matrix, factorizations, etc.
 *
 * TODO for large scale problems, build a subset of the graphs considered, using a tree or other structures
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
     * TODO Removes a Factor and on all the connected Nodes
     * list of factors, it is removed as well
     */
    void rmFactor(std::shared_ptr<Factor> &factor);
    /**
     * TODO Removes Node from list. TODO should we eliminate
     * all factors pointing to that node?
     */
    void rmNode(std::shared_ptr<Node> &node);
    void print(bool complete = false) const;


    //TODO
    void saveGraph() const;
    void loadGraph();
protected:
    /**
	 *  XXX is set better than vector for what we are using them?
	 *  Vector is much faster for direct access [], but needs allocation.
	 *  We are also interested on having indices on nodes and factors.
	 *  Set iterates ok O(1) and can remove elements nicely O(1).
	 *
	 *  For now we will use vectors, but we will mantain abstraction in case we need to change
     *
     */
    //std::unordered_set<std::shared_ptr<Node> >   nodes_;
    std::vector<std::shared_ptr<Node> >   nodes_, localNodes_;

    //std::unordered_set<std::shared_ptr<Factor> > factors_;
    std::vector<std::shared_ptr<Factor> > factors_, localFactors_;

    /**
     * Total accumulated dimensions on both the state (nodes)
     * and the observations (factors)
     */
    uint_t stateDim_, obsDim_;
    // This variable is for selecting subsets of nodes and factors stored on
    // localNodes and localFactors. TODO
    bool isHoleProblem_;
};



}

#endif /* FACTOR_Graph_HPP_ */
