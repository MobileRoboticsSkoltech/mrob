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

Factor::Factor(uint_t potNumberNodes):
        chi2_(0)
{
    neighbourNodes_.reserve( potNumberNodes );
}

Factor::~Factor()
{
    neighbourNodes_.clear();
}
