/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * gicp.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/gicp.hpp"

using namespace mrob;

GICP::GICP(const MatX &X, const MatX &Y, const MatX &CovX, const MatX &CovY):
        BaseTransf(X,Y), CovX_(CovX), CovY_(CovY)
{
}

GICP::~GICP()
{
}

int GICP::solve()
{
    return 1;
}
