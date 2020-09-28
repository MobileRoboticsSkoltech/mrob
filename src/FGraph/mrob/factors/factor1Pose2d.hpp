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
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef MROB_FACTOR1POSE2D_H
#define MROB_FACTOR1POSE2D_H

#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

using namespace mrob;


/**
 * The Factor1Poses2d is a vertex representing the distribution
 * of a nodePose2d, pretty much like an anchoring factor.
 *
 * The state is an observed RBT, coincident with the node state it is connected to.
 *
 * In particular, the residual of this factor is:
 *   r = obs-x
 */


namespace mrob{
    class Factor1Pose2d : public Factor
    {
    public:
        Factor1Pose2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                const Mat3 &obsInf);
        ~Factor1Pose2d() = default;

        void evaluate_residuals() override;
        void evaluate_jacobians() override;
        void evaluate_chi2() override;

        void print() const;

        const Eigen::Ref<const MatX> get_obs() const {return obs_;};
        const Eigen::Ref<const MatX1> get_residual() const {return r_;};
        const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
        const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

    protected:
        Mat31 obs_, r_; //and residuals
        Mat3 W_;//inverse of observation covariance (information matrix)
        Mat3 J_;//Jacobian
    };
}

#endif //MROB_FACTOR1POSE2D_H
