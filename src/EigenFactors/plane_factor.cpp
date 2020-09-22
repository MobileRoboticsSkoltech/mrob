/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
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
#include "mrob/SE3.hpp"

using namespace mrob;

PlaneFactor::PlaneFactor(const Mat4 &S, std::shared_ptr<Node> &nodeOrigin):
        Factor(0,0), //Dimension zero since this is a non-parametric factor. Also we don't known how many nodes will connect, so we set the second param to 0 (not-used)
        planeEstimation_(Mat41::Zero()),
        planeError_(0.0)
{
    planeNodes_.push_back(nodeOrigin);
    S_.emplace(nodeOrigin->get_id(), S);//TODO ids make sense? not really
}


void PlaneFactor::add_observation(const Mat4& S, std::shared_ptr<Node> &newNode)
{
    //neighbourNodes_ contain only those node to be optimized
    planeNodes_.push_back(newNode);
    S_.emplace(newNode->get_id(), S);
}

double PlaneFactor::estimate_plane()
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

void PlaneFactor::calculate_all_matrices_Q()
{
    Q_.clear();
    for (auto &element : S_)
    {
        // Find the corresponding transformation, on this pair<id, S>
        uint_t nodeId = element.first;
        SE3 T;
        // Use the corresponding matrix S
        Mat4 Q;
        Q.noalias() =  T.T() * element.second * T.T().transpose();
        Q_.emplace(nodeId, Q);
    }
}


Mat61 PlaneFactor::calculate_jacobian(uint_t nodeId)
{
    if (S_.count(nodeId) == 0) return Mat61::Zero();
    Mat61 jacobian;
    // calculate dQ/dxi for the submatrix S
    // XXX for now keep vectors S and Q and create a separate vector/map for adding optimization nodes or do not include others???
    Mat4 dQ, Q;// = matrixQ_[];

    // dQ / d xi(0) = [0
    //               -q3
    //                q2
    //                 0]
    dQ.setZero();
    dQ.row(1) << -Q.row(2);
    dQ.row(2) <<  Q.row(1);
    dQ += dQ.transpose().eval();
    jacobian(0) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(1) = [q3
    //                 0
    //                -q1
    //                 0]
    dQ.setZero();
    dQ.row(0) <<  Q.row(2);
    dQ.row(2) << -Q.row(0);
    dQ += dQ.transpose().eval();
    jacobian(1) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(2) = [-q2
    //                 q1
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << -Q.row(1);
    dQ.row(1) <<  Q.row(0);
    dQ += dQ.transpose().eval();
    jacobian(2) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(3) = [q4
    //                 0
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(0) << Q.row(3);
    dQ += dQ.transpose().eval();
    jacobian(3) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(4) = [0
    //                 q4
    //                 0
    //                 0]
    dQ.setZero();
    dQ.row(1) << Q.row(3);
    dQ += dQ.transpose().eval();
    jacobian(4) = planeEstimation_.dot(dQ*planeEstimation_);

    // dQ / d xi(5) = [0
    //                 0
    //                 q4
    //                 0]
    dQ.setZero();
    dQ.row(2) << Q.row(3);
    dQ += dQ.transpose().eval();
    jacobian(5) = planeEstimation_.dot(dQ*planeEstimation_);


    return jacobian;
}


void PlaneFactor::print() const
{
    std::cout << "Plane Eigen Factor" <<std::endl;
}
