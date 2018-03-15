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
#include <iostream>

using namespace fg;

Node::Node(uint_t dim, uint_t potNumberFactors):
        dim_(dim)
{
    neighbourFactors_.reserve( potNumberFactors );
}

Node::~Node()
{
    neighbourFactors_.clear();
}
bool Node::addFactor(std::shared_ptr<Factor> &factor)
{
    neighbourFactors_.push_back(factor);
    return true;
}

bool Node::rmFactor(std::shared_ptr<Factor> &factor)
{
    // exhaustive line search over the vector, this SHOULD be small, right?
    // still, it is very ineffcient O(n), but we have preferred using a vector
    // over a set or a map because we are gonna iterate over this container,
    // while it is not so clear that we would want to remove factors (although possible)
    std::vector<std::shared_ptr<Factor> >::iterator f;
    for (f = neighbourFactors_.begin(); f != neighbourFactors_.end(); ++f)
    {
        if (*f == factor)
            break;
    }
    neighbourFactors_.erase(f);
    return true;
}
