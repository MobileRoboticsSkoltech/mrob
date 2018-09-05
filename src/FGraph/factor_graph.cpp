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
#include "skmr/factor_graph.hpp"

using namespace skmr;


FGraph::FGraph(uint_t potNumberNodes, uint_t potNumberFactors) :
        stateDim_(0),obsDim_(0),isHoleProblem_(true)
{
    //For Sets:: max_load is 1, so it rehashes and augment the #bucklets in the same amount
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

bool FGraph::addFactor(std::shared_ptr<Factor> &factor)
{
	factor->setId(factors_.size()+1);//Starts at 1
	factors_.push_back(factor);
    auto list = factor->getNeighbourNodes();
    for( auto n: *list)
    {
        n->addFactor(factor);
    }
    obsDim_ += factor->getDim();
    return true;
}

bool FGraph::addNode(std::shared_ptr<Node> &node)
{
	node->setId(nodes_.size()+1);//XXX we assume that no node is deleted
	nodes_.push_back(node);
	stateDim_ += node->getDim();
	return true;
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
