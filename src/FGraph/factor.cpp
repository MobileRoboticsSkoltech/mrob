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

Factor::Factor(int id, int potNumberNodes) : id_(id)
{
    neighbourNodes_.reserve( potNumberNodes );
}

Factor::~Factor()
{
    neighbourNodes_.clear();
}
void Factor::addNeighbourNodes(std::shared_ptr<Node> &node)
{
    neighbourNodes_.push_back(node);
}

void Factor::rmNeighbourNodes(std::shared_ptr<Node> &node)
{
    // TODO programm me please
    assert(0 && "Factor::rmNeighbourNode: Not implemented yet");
}


