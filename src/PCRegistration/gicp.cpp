/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * Gicp.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include <Eigen/LU> // for inverse and determinant

#include <memory>
#include <iostream>
#include "mrob/pc_registration.hpp" // GICP function is defined here


using namespace mrob;

int PCRegistration::gicp(const Eigen::Ref<const MatX> X, const Eigen::Ref<const MatX> Y,
           const Eigen::Ref<const MatX> covX, const Eigen::Ref<const MatX> covY, SE3 &T, double tol)
{
    assert(X.rows() == 3  && "PCRegistration::Gicp: Incorrect sizing, we expect 3xN");
    assert(X.cols() >= 3  && "PCRegistration::Gicp: Incorrect sizing, we expect at least 3 correspondences (not aligned)");
    assert(Y.cols() == X.cols()  && "PCRegistration::Gicp: Same number of correspondences");
    uint_t N = X.cols();
    // TODO precalculation of T by reduced Arun
    // TODO different number of iterations and convergence criterion

    // Initialize Jacobian and Hessian
    Mat61 J = Mat61::Zero();
    Mat6 H = Mat6::Zero();
    uint_t iters = 0;
    double deltaUpdate = 1e3;
    do
    {
        J.setZero();
        H.setZero();
        // not vectoried operations (due to Jacobian)
        for ( uint_t i = 0; i < N ; ++i)
        {
            // 1) Calculate residual r = y - Tx and the inverse of joint covariance
            Mat31 Txi = T.transform(X.col(i));
            Mat31 r = Y.col(i) - Txi;
            Mat3 Li = (covY.block<3,3>(0,3*i) + T.R() * covX.block<3,3>(0,3*i) * T.R().transpose()).inverse();

            // 2) Calculate Jacobian for residual Jf = df1/d xi = r1' Li * Jr, where Jr = [(Tx)^ ; -I])
            Mat<3,6> Jr;
            Jr << hat3(Txi) , -Mat3::Identity();
            Mat<1,6> Ji = r.transpose() * Li * Jr;
            J += Ji;//Eigen manages this for us

            // 3) Hessian Hi ~ Jr' * Li * Jr
            Mat6 Hi = Jr.transpose() * Li * Jr;
            H += Hi;
        }
        // 4) Update Solution
        Mat61 dxi = -H.inverse()*J;
        T.update_lhs(dxi); //Left side update
        deltaUpdate = dxi.norm();
        iters++;

    }while(deltaUpdate > tol && iters < 20);

    return iters; // number of iterations
}
