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


#include "mrob/factor.hpp"

using namespace mrob;

Factor::Factor(uint_t dim, uint_t allNodesDim, uint_t potNumberNodes):
		id_(0), dim_(dim), allNodesDim_(allNodesDim), chi2_(0)
{
    // Child factor must specify dimensions
    neighbourNodes_.reserve( potNumberNodes );
}

Factor::~Factor()
{
    neighbourNodes_.clear();
}

