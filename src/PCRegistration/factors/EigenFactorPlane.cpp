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
    // for all nodes, calculate jacobian
    for (const auto &Qi: Q_)
    {
        Qi.second;
        //for( neighbournodes) TODO check ids of the local vector and the nodes ids, this can be a problme
        //     jacobian(0) = planeEstimation_.dot(dQ*planeEstimation_);
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
    if (S_.count(id) > 0)
    {
        allPlanePoints_.at(id).push_back(p);
    }
    // If EF has not observed point from the current Node, it creates:
    else
    {
        allPlanePoints_.emplace(id, std::vector<Mat31>{p});//TODO test this
    }
    numberPoints_++;

}


double EigenFactorPlane::estimate_plane()
{
    calculate_all_matrices_Q();
    accumulatedQ_ = Mat4::Zero();
    for (const auto &Qi: Q_)
    {
        accumulatedQ_ += Qi.second;
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

void EigenFactorPlane::calculate_all_matrices_Q()
{
    Q_.clear();
    for (auto &element : S_)
    {
        // Find the corresponding transformation, on this pair<id, S>
        factor_id_t nodeId = element.first;
        Mat4 T = this->neighbourNodes_[nodeId]->get_state();
        // Use the corresponding matrix S
        Mat4 Q;
        Q.noalias() =  T * element.second * T.transpose();
        Q_.emplace(nodeId, Q);
    }
}


void EigenFactorPlane::print() const
{
    std::cout << "Plane Eigen Factor" <<std::endl;
}
