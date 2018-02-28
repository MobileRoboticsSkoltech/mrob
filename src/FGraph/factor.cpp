/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor.cpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "factor.hpp"
#include <assert.h>

using namespace skmr;

Factor::Factor(unsigned int id, unsigned int potNumberNodes) : id_(id)
{
    neighbourNodes_.reserve( potNumberNodes );
}

Factor::~Factor()
{
    neighbourNodes_.clear();
}
void Factor::addNode(std::shared_ptr<Node> &node)
{
    neighbourNodes_.push_back(node);
}

void Factor::rmNode(std::shared_ptr<Node> &node)
{
    // TODO programm me please
    assert(0 && "Factor::rmNeighbourNode: Not implemented yet");
    // exhaustive line search over the vector, this SHOULD be small, right?
}


