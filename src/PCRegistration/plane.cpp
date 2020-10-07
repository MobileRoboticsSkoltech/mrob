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
 *
 * plane.cpp
 *
 *  Created on: Feb 1, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include "mrob/plane.hpp"
#include <iostream>
//#include <Eigen/SVD>
//#include <Eigen/LU>
#include <Eigen/Eigenvalues>



using namespace mrob;

Plane::Plane(uint_t timeLength):
        timeLength_(timeLength), isPlaneEstimated_(false), numberPoints_(0)
{
    allPlanePoints_.reserve(timeLength_);
    // there should be an individual reservation of points
    for (uint_t i = 0; i < timeLength_; ++i)
    {
        allPlanePoints_.push_back(std::vector<Mat31>());
        // estimated number of points per observation TODO
        allPlanePoints_[i].reserve(512);
    }
    matrixS_.reserve(timeLength_);
    matrixQ_.reserve(timeLength_);

    //Initialize matrices for calculating Hessian
    gradQ_.reserve(6);// one matrix for each dimension
    lieGenerativeMatrices_.reserve(6);

    //TODO create all generative matrices
    Mat4 G;
    G << 0, 0, 0, 0,
         0, 0, -1, 0,
         0, 1, 0, 0,
         0, 0, 0, 0;
    lieGenerativeMatrices_.push_back(G);
    G << 0, 0, 1, 0,
         0, 0, 0, 0,
         -1, 0, 0, 0,
         0, 0, 0, 0;
    lieGenerativeMatrices_.push_back(G);
    G << 0, -1, 0, 0,
         1, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
    lieGenerativeMatrices_.push_back(G);
    G << 0, 0, 0, 1,
         0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
    lieGenerativeMatrices_.push_back(G);
    G << 0, 0, 0, 0,
         0, 0, 0, 1,
         0, 0, 0, 0,
         0, 0, 0, 0;
    lieGenerativeMatrices_.push_back(G);
    G << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 1,
         0, 0, 0, 0;
    lieGenerativeMatrices_.push_back(G);
}

Plane::~Plane()
{
}

void Plane::reserve(uint_t d, uint_t t)
{
    if (t < timeLength_)
        allPlanePoints_[t].reserve(d);
}

void Plane::push_back_point(Mat31 &point, uint_t t)
{
    // XXX for now it only works on consecutive observations TODO if t> time length increase
    if (t < timeLength_)
    {
        //homogeneousPoint << point, 1.0;
        //allPlanePoints_[t].push_back(homogeneousPoint);
        allPlanePoints_[t].push_back(point);
        ++numberPoints_;
    }
}

std::vector<Mat31>& Plane::get_points(uint_t t)
{
    if (t >= timeLength_)
        return allPlanePoints_.back();
    return allPlanePoints_[t];
}

void Plane::clear_points()
{
    allPlanePoints_.clear();
}


double Plane::estimate_plane()
{
    calculate_all_matrices_S();
    calculate_all_matrices_Q();
    accumulatedQ_ = Mat4::Zero();
    for (Mat4 &Qi: matrixQ_)
    {
        accumulatedQ_ += Qi;
        //std::cout << Qi << std::endl;
    }

    // Eigen::SelfAdjointEigenSolver return sorted elements from min(0) to max (3)
    Eigen::SelfAdjointEigenSolver<Mat4> eigs(accumulatedQ_);
    planeEstimation_ = eigs.eigenvectors().col(0);
    //std::cout << eigs.eigenvectors() << "\n and solution \n" << planeEstimation_ <<  std::endl;
    //std::cout << "plane estimation error: " << eigs.eigenvalues() <<  std::endl;
    lambda_ = eigs.eigenvalues()(0);

    // new estimation is done, set flags TODO why?
    isPlaneEstimated_ = true;

    return lambda_;
}

// Is this really used anywhere?
double Plane::estimate_plane_incrementally(uint_t t)
{
    accumulatedQ_ -= matrixQ_[t];
    accumulatedQ_ +=  trajectory_->at(t).T() * matrixS_[t] * trajectory_->at(t).T().transpose();
    //std::cout << "new acc Q " << accumulatedQ_ << std::endl;
    Eigen::SelfAdjointEigenSolver<Mat4> eigs(accumulatedQ_);
    planeEstimation_ = eigs.eigenvectors().col(0);
    return eigs.eigenvalues()(0);
}

double Plane::get_error_incremental(uint_t t) const
{
    Mat4 Q = accumulatedQ_;
    //std::cout << "accumulated Q from prev version " << accumulatedQ_ << std::endl;
    Q -= matrixQ_[t];
    //std::cout << "substract: Q " << Q << std::endl;
    Q +=  trajectory_->at(t).T() * matrixS_[t] * trajectory_->at(t).T().transpose();
    //std::cout << "new acc Q " << Q << std::endl;
    Eigen::SelfAdjointEigenSolver<Mat4> eigs(Q);
    return eigs.eigenvalues()(0);
}

void Plane::calculate_all_matrices_S(bool reset)
{
    if (reset)
        matrixS_.clear();
    if (matrixS_.empty())
    {
        for (uint_t t = 0; t < timeLength_; ++t)
        {
            Mat4 S = Mat4::Zero();
            for ( Mat31 &p : allPlanePoints_[t])
            {
                Mat41 pHomog;
                pHomog << p , 1.0;
                S += pHomog * pHomog.transpose();
            }
            matrixS_.push_back(S);
        }
    }
}

Mat31 Plane::get_mean_point(uint_t t)
{
    assert(!matrixS_.empty() && "Plane::get_mean_point: S matrix empty");
    return matrixS_[t].topRightCorner<3,1>()/matrixS_[t](3,3);
}

void Plane::calculate_all_matrices_Q()
{
    matrixQ_.clear();
    for (uint_t t = 0; t < timeLength_; ++t)
    {
        Mat4 Q;
        Q.noalias() = trajectory_->at(t).T() * matrixS_[t] * trajectory_->at(t).T().transpose();
        matrixQ_.push_back(Q);
    }
}

Mat61 Plane::calculate_gradient(uint_t t)
{
    Mat61 jacobian;
    gradQ_.clear();//used for Hessian, we bookeep all calculated gradients

    // calculate dQ/dxi for the submatrix S
    Mat4 dQ, &Q = matrixQ_[t];

    // dQ / d xi(0) = [0
    //               -q3
    //                q2
    //                 0]
    dQ.setZero();
    dQ.row(1) << -Q.row(2);
    dQ.row(2) <<  Q.row(1);
    dQ += dQ.transpose().eval();
    gradQ_.push_back(dQ);
    jacobian(0) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(1) = [q3
    //                 0
    //                -q1
    //                 0]
    dQ.setZero();
    dQ.row(0) <<  Q.row(2);
    dQ.row(2) << -Q.row(0);
    dQ += dQ.transpose().eval();
    gradQ_.push_back(dQ);
    jacobian(1) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(2) = [-q2
    //                 q1
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << -Q.row(1);
    dQ.row(1) <<  Q.row(0);
    dQ += dQ.transpose().eval();
    gradQ_.push_back(dQ);
    jacobian(2) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(3) = [q4
    //                 0
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << Q.row(3);
    dQ += dQ.transpose().eval();
    gradQ_.push_back(dQ);
    jacobian(3) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(4) = [0
    //                 q4
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(1) << Q.row(3);
    dQ += dQ.transpose().eval();
    gradQ_.push_back(dQ);
    jacobian(4) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(5) = [0
    //                 0
    //                 q4
    //                 0]
    dQ.setZero();
    dQ.row(2) << Q.row(3);
    dQ += dQ.transpose().eval();
    gradQ_.push_back(dQ);
    jacobian(5) = planeEstimation_.dot(dQ*planeEstimation_);


    return jacobian;
}

Mat6 Plane::calculate_hessian(uint_t t)
{
    Mat6 hessian = Mat6::Zero();
    Mat4 ddQ, &Q = matrixQ_[t];

    // H = pi' * dd Q * pi, where dd Q = Bij + Bij' and
    // Bij = (Gi*Gj + Gj*Gi)Q*0.5 + Gi * dQ (previous gradient)
    // Upper triangular matrix
    for (uint_t i =0 ; i< 6 ; ++i)
    {
        for (uint_t j = i ; j< 6 ; ++j)
        {
            ddQ.setZero();
            ddQ = lieGenerativeMatrices_[i]*lieGenerativeMatrices_[j] + lieGenerativeMatrices_[j]*lieGenerativeMatrices_[i];
            //compound operator *= as in a*=b (this multiplies on the right: a*=b is equivalent to a = a*b)
            ddQ *= 0.5 * Q;
            ddQ += lieGenerativeMatrices_[i] * gradQ_[j];
            ddQ += ddQ.transpose().eval();
            hessian(i,j) = planeEstimation_.dot(ddQ*planeEstimation_);
        }
    }
    return hessian;
}

void Plane::print() const
{
    for (uint_t t = 0; t <  timeLength_; ++t)
    {
        std::cout << "Plane time = " << t << std::endl;
        for (Mat31 p : allPlanePoints_[t])
            std::cout << p(0) << ", " << p(1) << ", " << p(2) << std::endl;
    }
}

void Plane::reset()
{
    matrixS_.clear();
    matrixQ_.clear();
    accumulatedQ_ = Mat4::Zero();
    planeEstimation_ = Mat41::Zero();
}
