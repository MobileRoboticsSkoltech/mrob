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

using namespace fg;

Node::Node(unsigned int id, unsigned int potNumberFactors) : id_(id)
{
    neighbourFactors_.reserve( potNumberFactors );
}

Node::~Node()
{
    std::cout << "deleting node" << std::endl;
    neighbourFactors_.clear();
}
void Node::addFactor(std::shared_ptr<Factor> &factor)
{
    neighbourFactors_.push_back(factor);
}

void Node::rmFactor(std::shared_ptr<Factor> &factor)
{
    // TODO programm me please
    assert(0 && "Node::rmNeighbourFactors: Not implemented yet");
}
