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

GICP::GICP(const MatX &X, const MatX &Y, const MatX &covX, const MatX &covY):
        BaseTransf(X,Y), covX_(covX), covY_(covY)
{
    // Check for covariances data, stored as 3x3 blocks
    assert(covX.rows() == 3 && "GICP::GICP: Incorrect size of data, rows not 3");
    assert(covX.cols() == 3*N_ && "GICP::GICP: Incorrect size of data, cols");
}

GICP::~GICP()
{
}

int GICP::solve()
{
    return 1;
}
