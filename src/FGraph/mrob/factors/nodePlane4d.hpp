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
 * nodePlane4d.hpp
 *
 *  Created on: Oct 10, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef NODEPLANE4D_HPP_
#define NODEPLANE4D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{

class NodePlane4d : public Node
{
  public:
    /**
     * Node Plane 4d stands for the representation of planes as 4x1 vectors:
     *   pi = [n', d] = [nx, ny, nz, d], s.t. ||n|| =1
     *
     * For initialization, requires an initial estimation of the state in global coordinates.
     * TODO: relative formulation of planes might be more convinient.
     *
     * Note that the dimensionality of this node is 4, that is the DOF,
     * althought we know that the normal lies in S^2, so that shrinks the
     * DOF to 3 in total.
     */
    NodePlane4d(const Mat41 &initial_x, Node::nodeMode mode = STANDARD);
    virtual ~NodePlane4d()  override = default;
    /**
     * Update of 4d is simply addition and a clamping process to guarantee unit normal
     * pi'= clamped(pi + dx)
     *
     */
    virtual void update(const Eigen::Ref<const MatX1> &dx) override;
    virtual void update_from_auxiliary(const Eigen::Ref<const MatX1> &dx) override;
    virtual void set_state(const Eigen::Ref<const MatX> &x) override {state_ = x;};
    virtual void set_auxiliary_state(const Eigen::Ref<const MatX> &x) override {auxiliaryState_ = x;};
    virtual const Eigen::Ref<const MatX> get_state() const override {return state_;};
    virtual const Eigen::Ref<const MatX> get_auxiliary_state() const override {return auxiliaryState_;};
    virtual void print() const override;

  protected:
    Mat41 state_;
    Mat41 auxiliaryState_; //an auxiliary vector for undoing updates

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by the Eigen library

};
}
#endif /* NODEPLANE4D_HPP_ */
