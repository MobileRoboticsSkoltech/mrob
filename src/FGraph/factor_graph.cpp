/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor_graph.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include <iostream>
#include <mrob/factor_graph.hpp>


using namespace mrob;


FGraph::FGraph(uint_t potNumberNodes, uint_t potNumberFactors) :
        stateDim_(0),obsDim_(0),isHoleProblem_(true)
{
    //For Sets:: max_load is 1, so it rehashes and augment the #bucklets in the same amount
    std::cout << "number of factors = " << potNumberFactors << ", number of nodes = " << potNumberNodes;
    factors_.reserve(potNumberFactors);
    nodes_.reserve(potNumberNodes);
}
FGraph::~FGraph()
{
    // clear every node's list of neigbours factors
    for (auto n: nodes_)
        n->clear();
    factors_.clear();
    nodes_.clear();
}

bool FGraph::add_factor(std::shared_ptr<Factor> &factor)
{
	factor->set_id(factors_.size()+1);//Starts at 1 TODO why? we should not do this?
	factors_.push_back(factor);
    auto list = factor->get_neighbour_nodes();
    for( auto n: *list)
    {
        n->add_factor(factor);
    }
    obsDim_ += factor->get_dim();
    return true;
}

bool FGraph::add_node(std::shared_ptr<Node> &node)
{
	node->set_id(nodes_.size()+1);//XXX we assume that no node is deleted
	nodes_.push_back(node);
	stateDim_ += node->get_dim();
	return true;
}

std::shared_ptr<Node>& FGraph::get_node(uint_t key)
{
    // TODO key on a set or map?
    assert(key <= nodes_.size() && "FGraph::get_node: incorrect key");
    return nodes_[key-1];
}

void FGraph::print(bool completePrint) const
{
    std::cout << "Status of graph: " <<
            nodes_.size()  << "Nodes and " <<
            factors_.size() << "Factors." << std::endl;

    if(completePrint)
    {
        for (auto n : nodes_)
            n->print();
        for (auto f : factors_)
            f->print();
    }
}
