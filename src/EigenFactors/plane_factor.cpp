/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * plane_factor.cpp
 *
 *  Created on: Aug 16, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/plane_factor.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>

using namespace mrob;

PlaneFactor::PlaneFactor(const Mat4 &S, std::shared_ptr<Node> &nodeOrigin):
        Factor(0,0), //Dimension zero since this is a non-parametric factor. Also we don't known how many nodes will connect, so we set the second param to 0 (not-used)
        planeEstimation_(Mat41::Zero()),
        planeError_(0.0)
{
    neighbourNodes_.push_back(nodeOrigin);
    S_.emplace(nodeOrigin->get_id(), S);
}


void PlaneFactor::add_observation(const Mat4& S, std::shared_ptr<Node> &newNode)
{
    neighbourNodes_.push_back(newNode);
    S_.emplace(newNode->get_id(), S);
}

double PlaneFactor::estimate_plane()
{
    calculate_all_matrices_Q();
    accumulatedQ_ = Mat4::Zero();
    for (Mat4 &Qi: matrixQ_)
    {
        accumulatedQ_ += Qi;
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

void PlaneFactor::calculate_all_matrices_Q()
{
    matrixQ_.clear();
    for (auto element : S_)
    {
        // Find the correspoding transformation
        uint_t nodeId = element.first;
        SE3 T;
        // Use the correspoding matrix S
        Mat4 Q;
        Mat4 S = element.second;
        Q.noalias() =  T.T() * S * T.T().transpose();
        matrixQ_.push_back(Q);
    }
}

void PlaneFactor::print() const
{
    std::cout << "Plane Eigen Factor" <<std::endl;
}
