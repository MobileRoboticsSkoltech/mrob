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

using namespace fg;

Factor::Factor(unsign_t id, unsign_t potNumberNodes) : id_(id)
{
    neighbourNodes_.reserve( potNumberNodes );
}

Factor::~Factor()
{
    std::cout << "deleting factor" << std::endl;
    neighbourNodes_.clear();
}
void Factor::addNode(std::shared_ptr<Node> &node)
{
    neighbourNodes_.push_back(node);
}

void Factor::rmNode(std::shared_ptr<Node> &node)
{
    // exhaustive line search over the vector, this SHOULD be small, right?
    std::vector<std::shared_ptr<Node> >::iterator n;
    for (n = neighbourNodes_.begin(); n != neighbourNodes_.end(); ++n)
    {
        if (*n == node)
            break;
    }
    neighbourNodes_.erase(n);
}


