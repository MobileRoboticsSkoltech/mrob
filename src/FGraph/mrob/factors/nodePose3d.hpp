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
 * nodePose3d.hpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef NODEPOSE3D_HPP_
#define NODEPOSE3D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/node.hpp"

namespace mrob{

class NodePose3d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     * For 3D poses we use a transformation matrix 4x4
     * 
     * Note that the dimensionality of this node is 6, that is the DOF
     */
    NodePose3d(const Mat4 &initial_x);
    /**
     * Initialization directly on SE3 a matrix
     */
    NodePose3d(const SE3 &initial_x);
    virtual ~NodePose3d();
    /**
     * Left update operation corresponds to
     * T'=exp(dxi^)*T
     * x'=vee(ln(T'))
     */
    virtual void update(const Eigen::Ref<const MatX1> &dx);
    virtual void update_from_auxiliary(const Eigen::Ref<const MatX1> &dx);
    virtual void set_state(const Eigen::Ref<const MatX> &x);
    virtual void set_auxiliary_state(const Eigen::Ref<const MatX> &x);
    virtual const Eigen::Ref<const MatX> get_state() const {return state_.T();};
    virtual const Eigen::Ref<const MatX> get_auxiliary_state() const {return auxiliaryState_.T();};
    void print() const;

  protected:
    SE3 state_;
    SE3 auxiliaryState_; //an auxiliary vector for undoing updates

  public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen


};


}


#endif /* NODEPOSE3D_HPP_ */
