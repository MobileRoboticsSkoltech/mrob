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
 * nodeLandmark2d.hpp
 *
 *  Created on: Jul 27, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef NODELANDMARK2D_HPP_
#define NODELANDMARK2D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{

/**
 * Class Node Landmark 2d is a node whose state is a landmark position
 * l = [x,y]'.
 * Here we implement the necessary methods compliant with the node.hpp
 * interface
 */

class NodeLandmark2d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     */
    NodeLandmark2d(const Mat21 &initial_x, Node::nodeMode mode = STANDARD);
    virtual ~NodeLandmark2d()  override = default;

    virtual void update(const Eigen::Ref<const MatX1> &dx);
    virtual void update_from_auxiliary(const Eigen::Ref<const MatX1> &dx);
    virtual void set_state(const Eigen::Ref<const MatX> &x);
    virtual void set_auxiliary_state(const Eigen::Ref<const MatX> &x);
    virtual const Eigen::Ref<const MatX> get_state() const {return state_;};
    virtual const Eigen::Ref<const MatX> get_auxiliary_state() const {return auxiliaryState_;};
    void print() const;

  protected:
    Mat21 state_;
    Mat21 auxiliaryState_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen


};


}


#endif /* NODELANDMARK2D_HPP_ */
