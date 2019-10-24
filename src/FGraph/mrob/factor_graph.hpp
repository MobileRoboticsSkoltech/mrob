/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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

//#include <unordered_map>
#include <deque>//TODO change for long allocations

#include "mrob/factor.hpp"
#include "mrob/node.hpp"

namespace mrob{
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
 *  - Class Factor. here see factor.hpp for the conventions on residuals, observations, etc.
 *
 * XXX, actually key as addresses won't work in python interface. Better use id_t (uint)
 * Both data containers are stored in vectors (XXX prev unordered sets) whose keys are their addresses. By doing this, we can
 * iterate and quickly find elements in both data containers.
 *
 * Each problem instantaition should implement methods for solving the graph and storing the
 * necessary data, such as information matrix, factorizations, etc.
 *
 * TODO for large scale problems, build a subset of the graphs considered, using a tree or other structures
 */

class FGraph{
public:
    FGraph();
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
    bool add_factor(std::shared_ptr<Factor> &factor);
    /**
      * Adds a node if it was not already on the set.
      */
    bool add_node(std::shared_ptr<Node> &node);
    /**
     * TODO Removes a Factor and on all the connected Nodes
     * list of factors, it is removed as well
     */
    void rm_factor(std::shared_ptr<Factor> &factor);
    /**
     * TODO Removes Node from list. TODO should we eliminate
     * all factors pointing to that node?
     */
    void rm_node(std::shared_ptr<Node> &node);
    void print(bool complete = false) const;

    /**
     * get_node returns the node given the node id key, now a position on the data structure
     */
    std::shared_ptr<Node>& get_node(uint_t key);

    /**
     * get_node returns the node given the node id key, now a position on the data structure
     */
    std::shared_ptr<Factor>& get_factor(uint_t key);
    /**
     * Returns the chi2 corresponding to a particular factor. Use this only if the value has been updated recently
     * Mainly the purpose of this function is for testing
     */
    matData_t get_factor_chi2(uint_t key);
    /**
     * Evaluates the residual and returns the chi2 value
     */
    matData_t evaluate_factor_chi2(uint_t key);

    /**
     * FGraph information
     */
    uint_t number_nodes() {return nodes_.size();};
    uint_t number_factors() {return factors_.size();};

    //TODO
    void save_graph() const;
    void load_graph();

protected:
    /**
	 *  XXX is set better than vector for what we are using them?
	 *  Vector is much faster for direct access [], but needs allocation.
	 *  We are also interested on having indices on nodes and factors.
	 *  Set iterates ok O(1) and can remove elements nicely O(1).
	 *
	 *  For now we will use vectors, but we will maintain abstraction in case we need to change
     *
     */
    //std::unordered_set<std::shared_ptr<Node> >   nodes_;
    std::deque<std::shared_ptr<Node> >   nodes_; // no specific oder needed

    //std::unordered_set<std::shared_ptr<Factor> > factors_;
    std::deque<std::shared_ptr<Factor> > factors_; // no specific order needed

    /**
     * Total accumulated dimensions on both the state (nodes)
     * and the observations (factors)
     */
    uint_t stateDim_, obsDim_;
};



}

#endif /* FACTOR_Graph_HPP_ */
