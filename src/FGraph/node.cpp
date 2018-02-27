/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * node.cpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "node.hpp"
#include <assert.h>

using namespace skmr;

Node::Node(int id, int potNumberFactors) : id_(id)
{
    neighbourFactors_.reserve( potNumberFactors );
}

Node::~Node()
{
    neighbourFactors_.clear();
}
void Node::addNeighbourFactors(std::shared_ptr<Factor> &factor)
{
    neighbourFactors_.push_back(factor);
}

void Node::rmNeighbourFactors(std::shared_ptr<Factor> &factor)
{
    // TODO programm me please
    assert(0 && "Node::rmNeighbourFactors: Not implemented yet");
}
