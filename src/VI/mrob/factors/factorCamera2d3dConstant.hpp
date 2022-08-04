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
 * factorCamera2d3dConstant.hpp
 *
 *  Created on: Aug 3, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef FACTORCAMERA2D3DCONSTANT_HPP_
#define FACTORCAMERA2D3DCONSTANT_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"
#include "mrob/factors/nodeCameraPinhole.hpp"

namespace mrob{

/**
 * The FactorCamera2d3dConstant is a vertex (factor) connecting a point in 3D (constant),
 * which gets projected into the image plane, by a PinHole camera model (constant), into a 2d point (in pixels)
 *
 * Input:
 *  - Observation: point in pixels. Distortion must be corrected beforehand
 *  - nodePose: this poses expressed the transformation between the points in 3D to the camera frame
 *  - nodeCamera: node for the camera Pinhole model. In principle this will be an anchor factor (we dont evaluate J)
 *  - nodePoint: point in 3D. Anchor factor (contant)
 *  -
 *
 */

class FactorCamera2d3dConstant : public Factor
{
public:
    FactorCamera2d3dConstant(const Mat21 &observation,
                            std::shared_ptr<Node> &nodePose,
                            std::shared_ptr<Node> &nodePoint,
                            std::shared_ptr<Node> &nodeCamera,
                            const Mat2 &obsInf,
                            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC
                            );
    ~FactorCamera2d3dConstant() override = default;
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

    virtual const Eigen::Ref<const MatX> get_obs() const {return obs_;};
    virtual const Eigen::Ref<const MatX1> get_residual() const {return r_;};
    virtual const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
    virtual const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

protected:
    Mat21 obs_, r_;
    Mat2 W_;
    Mat<2,6> J_;
    Mat31 P_camera_coord_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};

} // namespace

#endif /* FACTORCAMERA2D3DCONSTANT_HPP_ */
