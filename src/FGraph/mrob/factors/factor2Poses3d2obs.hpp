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
 * factor2Poses3d2obs.hpp
 *
 *  Created on: Nov 9, 2021
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR2POSES3D2OBS_HPP_
#define FACTOR2POSES3D2OBS_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

    /**
     * The Factor2Poses3d2obs is a variant of the 2 poses factor2Poses3d2
     * This is a special factor that connects 2 unknown poses, origin and target
     * by two different observations. This is usually useful for calibration problems,
     * where we need to estimate unknon poses of the form:
     *
     *    X * Obs1 * Y = Obs2
     *
     * The state is an observer RBT, and as we have commented, we need to specify
     * the two Nodes that the factor is connecting, which are provided by their
     * shared_ptr's.
     * We provide the node's Id to get the correspondent Jacobian
     *
     * The convention in the library r = f(x) - z.
     *
     *
     *
     *
     *
     * The residual is:
     *   r = Ln ( T_o * T_obs * T_t * T_obs2^-1)
     *
     * Constructor functions will be overloaded to include the pointers of the nodes,
     * The convention is from node origin, we observe node destination,
     * such that: Factor2Poses3d(nodeOrigin, nodeTarget, ...
     *
     * The observations relate a pair of nodes. The order matters, since this will
     * affect the order on the Jacobian block matrix
     */

class Factor2Poses3d2obs : public Factor
{
  public:
    Factor2Poses3d2obs(const Mat4 &observation, const Mat4 &observation2, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    Factor2Poses3d2obs(const SE3 &observation, const SE3 &observation2, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor2Poses3d2obs() override = default;
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
    SE3 Tobs_, Tobs2_; // Transformation from observation. NOTE: In Xorigin frame
    Mat61 r_; //and residuals
    SE3 Tr_; // Residual Transformation
    Mat6 W_;//inverse of observation covariance (information matrix)
    Mat<6,12> J_;//Joint Jacobian

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3D2OBS_HPP_ */
