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

GICP::GICP(const std::shared_ptr<MatX> &X, const std::shared_ptr<MatX> &Y, MatX &CovX, MatX &CovY):
        BaseTransf(X,Y)
{
}

GICP::~GICP()
{
}

int GICP::solve()
{
    return 1;
}
