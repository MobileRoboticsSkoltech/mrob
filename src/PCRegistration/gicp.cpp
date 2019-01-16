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
    // TODO precalculation of T by reduced Arun
    // TODO different number of iterations and convergence criterion

    // Initialize Jacobian and Hessian
    Mat<6,1> J = Mat<6,1>::Zero();
    Mat6 H = Mat6::Zero();

    // not vectoried operations (due to Jacobian)
    for ( uint_t i = 0; i < N_ ; ++i)
    {
        // 1) Calculate residual r = y - Tx and the inverse of joint covariance
        Mat31 Txi = T_.transform(X_.col(i));
        Mat31 r = Y_.col(i) - Txi;
        Mat3 Li = (covY_.block<3,3>(0,3*i) + T_.R() * covX_.block<3,3>(0,3*i) * T_.R().transpose()).inverse();

        // 2) Calculate Jacobian for residual Jf = df1/d xi = r1' Li * Jr, where Jr = [(Tx)^ ; -I])
        Mat<3,6> Jr;
        Jr << hat3(Txi) , -Mat3::Identity();
        Mat<1,6> Ji = r.transpose() * Li * Jr;
        J += Ji;

        // 3) Hessian Hi ~ Jr' * Li * Jr
        Mat6 Hi = Jr.transpose() * Li * Jr;
        H += Hi;
    }
    // 4) Update Solution
    Mat61 dxi = -H.inverse()*J;
    T_.update(dxi); //Left side update
    return 1; // number of iterations
}
