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
 * factor2Poses3d.hpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR2POSES3D_HPP_
#define FACTOR2POSES3D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The Factor2Poses3d is a vertex representing the distribution between
 * two nodePose3d, that is, it is expressing a Rigid Body Transformation
 * between two poses.
 *
 * The state is an observer RBT, and as we have commented, we need to specify
 * the two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's.
 * We provide the node's Id to get the correspondent Jacobian
 *
 * The convention in the library r = f(x) - z.
 *
 * In this particular factor, we will follow a similar convention as in odometry 2d,
 * where we 'observe' the last pose, and thus, the relation between the transformation of poses is:
 *   T_o * T_obs = T_t
 *
 *
 *
 * T_o is the transformation encoded by the 3D pose 'origin'. Also note that the
 * transformation from a pose (Exp(x_o) transforms point in the local 'origin' frame to the world reference.
 * T_t target transformation from pose x_t
 * T_obs observation transformation from pose obs (observed from origin)
 *
 * and the residual is thus:
 *   r = Ln ( T_o * T_obs * T_t^-1 )
 *
 * (equivalent to odometry 2d x_origin + observation - x_target)
 *
 *
 * (it could also be formulated as T_o^-1*T_t*Tob^-1, but the former way is more intuitive
 *
 * Constructor functions will be overloaded to include the pointers of the nodes,
 * The convention is from node origin, we observe node destination,
 * such that: Factor2Poses3d(nodeOrigin, nodeTarget, ...
 *
 * The observations relate a pair of nodes. The order matters, since this will
 * affect the order on the Jacobian block matrix
 */

class Factor2Poses3d : public Factor
{
  public:
    Factor2Poses3d(const Mat4 &observation, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf, bool updateNodeTarget=false);
    Factor2Poses3d(const SE3 &observation, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf, bool updateNodeTarget=false);
    ~Factor2Poses3d();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    virtual void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;

    virtual void print() const;

    virtual const Eigen::Ref<const MatX> get_obs() const {return Tobs_.T();};
    virtual const Eigen::Ref<const MatX1> get_residual() const {return r_;};
    virtual const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
    virtual const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

  protected:
    // The Jacobians' correspondant nodes are ordered on the vector<Node>
    // being [0]->J_origin and [1]->J_target
    // declared here but initialized on child classes
    SE3 Tobs_; // Transformation from observation. NOTE: In Xorigin frame
    Mat61 r_; //and residuals
    SE3 Tr_; // Residual Transformation
    Mat6 W_;//inverse of observation covariance (information matrix)
    Mat<6,12> J_;//Joint Jacobian

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3D_HPP_ */
