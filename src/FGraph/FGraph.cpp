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


using namespace fg;


FGraph::FGraph(unsigned int potNumberNodes, unsigned int potNumberFactors) :
        factorCount_(0), nodeCount_(0)
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
    auto res = factors_.emplace(factor->getId(), factor);
    return res.second;
}
bool FGraph::addNode(std::shared_ptr<Node> &node)
{
    auto res = nodes_.emplace(node->getId(), node);
    return res.second;
}
