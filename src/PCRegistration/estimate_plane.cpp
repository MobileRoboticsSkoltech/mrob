/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * estimate_plane.cpp
 *
 *  Created on: Dec 28, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/estimate_plane.hpp"
#include <Eigen/Eigenvalues>

using namespace mrob;

Mat41 estimate_plane_centered(const Eigen::Ref<const MatX> X);

Mat41 mrob::estimate_plane(const Eigen::Ref<const MatX> X)
{
    // Initialization
    assert(X.cols() == 3  && "Estimate_plane: Incorrect sizing, we expect Nx3");
    assert(X.rows() >= 3  && "Estimate_plane: Incorrect sizing, we expect at least 3 correspondences (not aligned)");

    // Plane estimation, centered approach
    return estimate_plane_centered(X);
}

//local function, we also will test the homogeneous plane estimation
Mat41 estimate_plane_centered(const Eigen::Ref<const MatX> X)
{
    uint_t N = X.rows();
    // Calculate center of points:
    Mat13 c =  X.colwise().sum();
    c /= (double)N;


    MatX qx = X.rowwise() - c;
    Mat3 C = qx.transpose() * qx;


    Eigen::SelfAdjointEigenSolver<Mat3> eigs;
    eigs.computeDirect(C);

    Mat31 normal = eigs.eigenvectors().col(0);

    matData_t d = - c*normal;

    Mat41 plane;
    plane << normal, d;
    // error = eigs.eigenvalues()(0);
    return plane;

}



Mat31 mrob::estimate_normal(const Eigen::Ref<const MatX> X)
{
    Mat41 res = estimate_plane(X);
    return res.head(3);
}


Mat31 mrob::estimate_centroid(const Eigen::Ref<const MatX> X)
{
    // Initialization
    assert(X.cols() == 3  && "Estimate_centroid: Incorrect sizing, we expect Nx3");
    assert(X.rows() >= 3  && "Estimate_centroid: Incorrect sizing, we expect at least 3 correspondences (not aligned)");

    uint_t N = X.rows();
    Mat13 c =  X.colwise().sum();
    c /= (double)N;
    return c;
}
