/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * node.cpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/node.hpp"

using namespace mrob;

Node::Node(uint_t dim, uint_t potNumberFactors):
		 id_(0), dim_(dim)
{
    neighbourFactors_.reserve( potNumberFactors );
}

Node::~Node()
{
    neighbourFactors_.clear();
}
bool Node::add_factor(std::shared_ptr<Factor> &factor)
{
    neighbourFactors_.push_back(factor);
    return true;
}

bool Node::rm_factor(std::shared_ptr<Factor> &factor)
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

// support function for 2D poses
double mrob::wrap_angle(double angle)
{
    double pi2 = 2 * M_PI;

    while (angle < -M_PI) angle += pi2;
    while (angle >= M_PI) angle -= pi2;

    return angle;
}
