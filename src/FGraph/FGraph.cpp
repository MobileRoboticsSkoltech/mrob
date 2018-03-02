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


FGraph::FGraph(uint_t potNumberNodes, uint_t potNumberFactors)
{
    //max_load is 1, so it rehashes and augment the #bucklets in the same amount
    factors_.reserve(potNumberFactors);
    nodes_.reserve(potNumberNodes);
}
FGraph::~FGraph()
{
    nodes_.clear();
    factors_.clear();
}

bool FGraph::addFactor(std::shared_ptr<Factor> &factor)
{
    auto res = factors_.insert(factor);
    return res.second;
}
bool FGraph::addNode(std::shared_ptr<Node> &node)
{
    std::cout << "entering add Node" << std::endl;
    auto res = nodes_.insert(node);
    return res.second;
}
void FGraph::rmFactor(std::shared_ptr<Factor> &factor)
{
    // remove from any extra thing
    factors_.erase(factor);
}
void FGraph::rmNode(std::shared_ptr<Node> &node)
{
    nodes_.erase(node);
}
void FGraph::printStatus() const
{
    std::cout << "Status of graph: " <<
            nodes_.size()  << "Nodes and " <<
            factors_.size() << "Factors." << std::endl;
}
