/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * arun.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include <Eigen/LU>
#include <Eigen/SVD>

#include <memory>
#include <iostream>
#include "mrob/pc_registration.hpp"

using namespace mrob;
using namespace Eigen;

int PCRegistration::Arun(const Ref<const MatX> X, const Ref<const MatX> Y, SE3 &T)
{
    assert(X.rows() == 3  && "PCRegistration::Arun: Incorrect sizing, we expect 3xN");
    assert(X.cols() >= 3  && "PCRegistration::Arun: Incorrect sizing, we expect at least 3 correspondences (not aligned)");
    assert(Y.cols() == X.cols()  && "PCRegistration::Arun: Same number of correspondences");
    uint_t N = X.cols();
    /** Algorithm:
     *  1) calculate centroids cx = sum x_i. cy = sum y_i
     *  2) calculate dispersion from centroids qx = x_i - cx
     *  3) calculate matrix H = sum qx_i * qy_i^T
     *  4) svd decomposition: H = U*D*V'
     *      4.5) look for co-linear solutions, that is 2 of the 3 singular values are equal
     *  5) Calculate the rotation solution R = V*U'
     *      5.5) check for correct solution (det = +1) or reflection (det = -1)
     *      step 5.5 is actually unnecessary IF applying Umeyama technique
     *  6) calculate translation as: t = cy - R * cx
     */
    // We have already asserted in base_T that they are 3xN matrices. (and the same length).

    //std::cout << "X: \n" << X << "\nY:\n" << Y << std::endl;
    // 1) calculate centroids cx = E{x_i}. cy = E{y_i}
    //More efficient than creating a matrix of ones when on Release mode (not is Debug mode)
    Mat31 cxm = X.rowwise().sum();
    cxm /= (double)N;
    Mat31 cym = Y.rowwise().sum();
    cym /= (double)N;

    // 2)  calculate dispersion from centroids qx = x_i - cx
    MatX qx = X.colwise() - cxm;
    MatX qy = Y.colwise() - cym;


    // 3) calculate matrix H = sum qx_i * qy_i^T
    Mat3 H = qx * qy.transpose();

    // 4) svd decomposition: H = U*D*V'
    JacobiSVD<Matrix3d> SVD(H, ComputeFullU | ComputeFullV);//Full matrices indicate Square matrices

    //test: prints results so far
    /*std::cout << "Checking matrix SVD: \n" << SVD.singularValues() <<
                 ",\n U = " << SVD.matrixU() <<
                 ",\n V = " << SVD.matrixV() << std::endl;*/


    // 4.5) look for co-linear solutions, that is 2 of the 3 singular values are equal
    double l_prev = SVD.singularValues()(0), l;
    for(int i =1; i < 3; ++i)
    {
        l = SVD.singularValues()(i);
        if (fabs(l - l_prev) < 1e-6)

            return 0; //they are co-linear, there exist infinite transformations
        else
            l_prev = l;//this works because we assume that they singular values are ordered.
    }


    // 5) Calculate the rotation solution R = V*U'
    Mat3 R = SVD.matrixV() * SVD.matrixU().transpose();

    // 5.5) check for correct solution (det = +1) or reflection (det = -1)
    // that is, solve the problem for co-planar set of points and centroid, when is l1 > l2 > l3 = 0
    // Since H = D1*u1*v1' + D2*u2*v2' + D3*u3*v3',    and D3 = 0, we can swap signs in V
    // such as Vp = [v1,v2,-v3] and the solution is still minimal, but we want a valid rotation R \in SO(3)
    if (R.determinant() < 0.0 )
    {
        Mat3 Vn;
        Vn << SVD.matrixV().topLeftCorner<3,2>(), -SVD.matrixV().topRightCorner<3,1>();
        R << Vn * SVD.matrixU().transpose();
        //std::cout << "R value = " << R << std::endl;
    }

    // 6) calculate translation as: t = cy - R * cx
    Mat31 t = cym - R*cxm;
    //std::cout << "t = " << t << std::endl;

    // 7) return result
    T.ref2T() << R, t,
                 0,0,0,1;

    return 1;
}
