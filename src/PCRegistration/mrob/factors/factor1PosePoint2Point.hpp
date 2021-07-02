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
 * factor1PosePoint2Point.hpp
 *
 *  Created on: Dec 29, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTORS_FACTOR1POSEPOINT2POINT_HPP_
#define FACTORS_FACTOR1POSEPOINT2POINT_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Factor1PoseToPpoint2Point is a vertex in the Fgraph library that models
 * the distribution of observations of a 3D pose. The purpose of this factors is to estimate
 * the relative transformation from two different sets of point clouds, given some planar constraaints.
 *
 * Observations
 *  - Point x
 *  - Point y, from reference y
 *
 * State to estimate is 3D pose y^T_x.
 *
 * The residual, as a convention in the library is:
 *   r = f(x) = (Tx - y) -> 0 if the points are coincident.
 *
 * The Jacobian is then
 *  dr = [-(Tx)^, I]
 *
 * TODO: correctly characterize the covariance, since this is a contribution from 2 rv
 */

class Factor1PosePoint2Point : public Factor
{
  public:
    Factor1PosePoint2Point(const Mat31 &z_point_x, const Mat31 &z_point_y,  std::shared_ptr<Node> &node,
            const Mat1 &obsInf, Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~Factor1PosePoint2Point();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    virtual void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     * J = dr/dxi = [-Tx^. I]
     */
    virtual void evaluate_jacobians() override;
    virtual void evaluate_chi2() override;

    virtual void print() const;

    virtual const Eigen::Ref<const MatX> get_obs() const {return r_;};
    virtual const Eigen::Ref<const MatX1> get_residual() const {return r_;};
    virtual const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
    virtual const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

  protected:
    Mat31 z_point_x_, z_point_y_, Tx_;
    Mat31 r_;
    Mat3 W_;
    Mat<3,6> J_;
};

}



#endif /* FACTORS_FACTOR1POSEPOINT2POINT_HPP_ */
