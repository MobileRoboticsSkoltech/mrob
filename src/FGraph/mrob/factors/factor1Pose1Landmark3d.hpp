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
 * factor1Pose1Landmark3d.hpp
 *
 *  Created on: March 17, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR1POSE1LANDMARK3D_HPP_
#define FACTOR1POSE1LANDMARK3D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The factor1Pose1Landmark3d is a vertex representing the distribution between
 * a Rigid Body Transformation encoding a 3D pose and a Landmark, a 3D point
 *
 * The observation is a 3D point, in the local frame of the current 3D pose.
 * The two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's, are:
 *  - 1 Pose3d
 *  - 1 Landmark3d
 * We provide the node's Id to get the correspondent Jacobian
 *
 *
 * In particular, the relation between the transformation of poses is:
 *   z = T^{-1}*l
 *
 * z is a 3d point with the observations in the local frame T
 * T is the transformation encoded by the 3D pose, the local frame.
 * l is a 3d point encoding the landmark position
 *
 * and the residual is thus:
 *   r = T^{-1}l - z
 *
 *
 * Constructor functions will be overloaded to include the pointers of the nodes,
 * The convention is 3d pose, we observe node destination,
 * such that: Factor1Pose1Landmark3d(nodePose, nodeLandmark, ...
 *
 * The observations relate a pair of nodes. The order matters, since this will
 * affect the order on the Jacobian block matrix
 */

class Factor1Pose1Landmark3d : public Factor
{
  public:
    Factor1Pose1Landmark3d(const Mat31 &observation, std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodeLandmark, const Mat3 &obsInf, bool initializeLandmark=false,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1Pose1Landmark3d() override = default;
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

    MatRefConst get_obs() const {return obs_;};
    VectRefConst get_residual() const {return r_;};
    MatRefConst get_information_matrix() const {return W_;};
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const {return J_;};

  protected:
    Mat31 obs_, r_, landmark_;
    SE3 Tinv_;
    Mat3 W_;//inverse of observation covariance (information matrix)
    Mat<3,9> J_;//Joint Jacobian
    bool reversedNodeOrder_;//flag to keep order when building the adjacency matrix. This should be transparent for the user

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* Factor1Pose1Landmark3d_HPP_ */
