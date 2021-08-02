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
 * plane_factor.cpp
 *
 *  Created on: Aug 16, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factors/EigenFactorPlane.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include "mrob/SE3.hpp"
#include <vector>

using namespace mrob;

EigenFactorPlane::EigenFactorPlane(Factor::robustFactorType robust_type):
        Factor(0,0, robust_type), //Dimension zero since this is a non-parametric factor. Also we don't known how many nodes will connect, so we set the second param to 0 (not-used)
        planeEstimation_{Mat41::Zero()},
        planeError_{0.0},
        numberPoints_{0}
{
}

void EigenFactorPlane::evaluate_residuals()
{
    this->estimate_plane();
}

void EigenFactorPlane::evaluate_jacobians()
{
    // Assumes residuals evaluated beforehand
    J_.clear();
    H_.clear();
    // for all nodes, calculate jacobian
    uint_t nodeIdLocal = 0;
    for (auto &Qt: Q_)
    {
        Mat61 jacobian = Mat61::Zero();
        Mat4 dQ = Mat4::Zero();
        for (uint_t i = 0 ; i < 6; i++)
        {
            dQ = SE3GenerativeMatrix(i)*Qt + Qt*SE3GenerativeMatrix(i);
            jacobian(i) = planeEstimation_.dot(dQ*planeEstimation_);

        }
        J_.push_back(jacobian);

        //now calculate Hessian here
        Mat6 hessian = Mat6::Zero();
        Mat4 ddQ;
        for (uint_t i =0 ; i< 6 ; ++i)
        {
            for (uint_t j = i ; j< 6 ; ++j)
            {
                ddQ.setZero();
                ddQ = SE3GenerativeMatrix(i)*SE3GenerativeMatrix(j) + SE3GenerativeMatrix(j)*SE3GenerativeMatrix(i);
                //compound operator *= as in a*=b (this multiplies on the right: a*=b is equivalent to a = a*b)
                ddQ *= 0.5 * Qt;
                ddQ += SE3GenerativeMatrix(i) * dQ;
                ddQ += ddQ.transpose().eval();
                hessian(i,j) = planeEstimation_.dot(ddQ*planeEstimation_);
            }
        }
        H_.push_back(hessian);

        nodeIdLocal++;
    }
}

void EigenFactorPlane::evaluate_chi2()
{
    chi2_ = planeError_;//not really an evaluation...should we do again?
}

void EigenFactorPlane::add_point(const Mat31& p, std::shared_ptr<Node> &node)
{
    // Pose has been observed, data has been initialized and we simply add point
    auto id = node->get_id();
    if (reverseNodeIds_.count(id) > 0)
    {
        uint_t localId = reverseNodeIds_[id];
        allPlanePoints_.at(localId).push_back(p);
    }
    // If EF has not observed point from the current Node, it creates:
    else
    {
        allPlanePoints_.emplace_back(std::deque<Mat31>());
        neighbourNodes_.push_back(node);
        nodeIds_.push_back(id);
        uint_t localId = allPlanePoints_.size();
        reverseNodeIds_.emplace(id, localId);
        // S and Q are built later, no need to create an element.
    }
    numberPoints_++;

}


double EigenFactorPlane::estimate_plane()
{
    calculate_all_matrices_S();
    calculate_all_matrices_Q();
    accumulatedQ_ = Mat4::Zero();
    for (auto &Qt: Q_)
    {
        accumulatedQ_ += Qt;
        //std::cout << Qi << std::endl;
    }

    // Only needs Lower View from Q (https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html)
    Eigen::SelfAdjointEigenSolver<Mat4> es(accumulatedQ_);
    planeEstimation_ = es.eigenvectors().col(0);
    //std::cout << es.eigenvectors() << "\n and solution \n" << planeEstimation_ <<  std::endl;
    //std::cout << "plane estimation error: " << es.eigenvalues() <<  std::endl;
    planeError_ = es.eigenvalues()(0);

    return planeError_;
}

void EigenFactorPlane::calculate_all_matrices_S(bool reset)
{
    if (reset)
        S_.clear();
    if (S_.empty())
    {
        for (auto &vectorPoints: allPlanePoints_)
        {
            Mat4 S = Mat4::Zero();
            for (Mat31 &p : vectorPoints)
            {
                Mat41 pHomog;
                pHomog << p , 1.0;
                S += pHomog * pHomog.transpose();
            }
            S_.push_back(S);
        }
    }
}

void EigenFactorPlane::calculate_all_matrices_Q()
{
    Q_.clear();
    uint_t nodeIdLocal = 0;
    for (auto &S : S_)
    {
        Mat4 T = this->neighbourNodes_[nodeIdLocal]->get_state();
        // Use the corresponding matrix S
        Mat4 Q;
        Q.noalias() =  T * S * T.transpose();
        Q_.push_back(Q);
        nodeIdLocal++;
    }
}

Mat31 EigenFactorPlane::get_mean_point(factor_id_t id)
{
    assert(!S_.empty() && "EigenFactorPlane::get_mean_point: S matrix empty");
    auto localId = reverseNodeIds_.at(id);
    return S_[localId].topRightCorner<3,1>()/S_[localId](3,3);
}

void EigenFactorPlane::print() const
{
    std::cout << "Plane Eigen Factor" <<std::endl;
}
