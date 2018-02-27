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


using namespace skmr;


FGraph::FGraph()
{
    factors_.reserve(512);
}
FGraph::~FGraph()
{
    nodes_.clear();
    factors_.clear();
}

void FGraph::addFactor(std::shared_ptr<Factor> &factor)
{
    factors_.emplace(factor->getId(), factor);
}
void FGraph::addNode(std::shared_ptr<Node> &node)
{
    nodes_.push_back(node);
}
