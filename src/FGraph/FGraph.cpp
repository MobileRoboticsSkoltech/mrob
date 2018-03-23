/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraph.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "FGraph.hpp"
#include <iostream>

using namespace fg;


FGraph::FGraph(uint_t potNumberNodes, uint_t potNumberFactors) :
		isHoleProblem_(false)
{
    //For Ste:: max_load is 1, so it rehashes and augment the #bucklets in the same amount
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
	return true;
    /* XXX if changes to Set, to be removed
    auto res = factors_.insert(factor);
    if (res.second)
    {
        auto list = factor->getNeighbourNodes();
        for( auto n: *list)
        {
            n->addFactor(factor);
        }
        return true;
    }
    return false;*/
}
bool FGraph::addNode(std::shared_ptr<Node> &node)
{
	node->setId(nodes_.size()+1);
	nodes_.push_back(node);
	return true;
    //auto res = nodes_.insert(node);
    //return res.second;
}
void FGraph::rmFactor(std::shared_ptr<Factor> &factor)
{
	assert(0);
    /* For sets, not implemented for vectors
	// remove from any extra thing
    auto list = factor->getNeighbourNodes();
    for( auto n: *list)
    {
        n->rmFactor(factor);//its an exhaustive search...TODO remove?
    }
    factors_.erase(factor);*/
}
void FGraph::rmNode(std::shared_ptr<Node> &node)
{
	assert(0);
    // TODO Factors associated to this node should be removed
    //nodes_.erase(node);
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
