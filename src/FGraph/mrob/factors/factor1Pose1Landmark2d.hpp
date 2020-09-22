/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 * factor1Pose1Landmark2d.hpp
 *
 *  Created on: Jul 27, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTOR1POSE1LANDMARK2D_HPP_
#define FACTOR1POSE1LANDMARK2D_HPP_



#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The factor1Pose1Landmark2d is a vertex representing the distribution between
 * a 2D pose and a Landmark, a 2D point. The observation consist of Range and Bearing.
 *
 * The observation is a 2D point, in the local frame of the current 2D pose.
 * The two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's, are:
 *  - 1 Pose2d
 *  - 1 Landmark2d
 * We provide the node's Id to get the correspondent Jacobian
 *
 *
 * In particular, the relation between the transformation of poses is:
 *   z = h(x)
 *
 * z = [range,bearing] is a 2d vector, composed of a range and bearing in the local frame.
 * l is a 2d point encoding the landmark position l = [x,y]
 *
 * and the residual is thus:
 *   r = T-1*l - z
 *
 *
 * Constructor functions will be overloaded to include the pointers of the nodes,
 * The convention is 2d pose, we observe node destination,
 * such that: Factor1Pose1Landmark2d(nodePose, nodeLandmark, ...
 *
 * The observations relate a pair of nodes. The order matters, since this will
 * affect the order on the Jacobian block matrix
 */

class Factor1Pose1Landmark2d : public Factor
{
  public:
    Factor1Pose1Landmark2d(const Mat21 &observation, std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodeLandmark, const Mat2 &obsInf, bool initializeLandmark=false);
    ~Factor1Pose1Landmark2d();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const;

    const Eigen::Ref<const MatX> get_obs() const {return obs_;};
    const Eigen::Ref<const MatX1> get_residual() const {return r_;};
    const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
    const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

  protected:
    Mat21 obs_, r_, landmark_;
    Mat31 state_;
    matData_t dx_, dy_, q_; // increments from pose to landmark (x,y) and q squared L2 distance
    Mat2 W_;//inverse of observation covariance (information matrix)
    Mat<2,5> J_;//Joint Jacobian, Block will depend on order
    bool reversedNodeOrder_;//flag to keep order when building the adjacency matrix. This should be transparent for the user

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}


#endif /* FACTOR1POSE1LANDMARK2D_HPP_ */
