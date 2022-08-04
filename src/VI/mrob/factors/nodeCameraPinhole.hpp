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
 * nodeCameraPinhole.hpp
 *
 *  Created on: Aug 4, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef NODECAMERAPINHOLE_HPP_
#define NODECAMERAPINHOLE_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/node.hpp"

namespace mrob{

class NodeCameraPinhole : public Node
{
public:
    /**
     * NodeCameraPinhole requires the intrinsic parameters from the pinhole model.
     * By default, we assume this is an anchor (constant) factor, but it can be changed for calibration purposes
     *
     * k_state: vector containing [fx, fy, cx, cy]
     */
    NodeCameraPinhole(const Mat41 k_state, Node::nodeMode mode = ANCHOR);
    ~NodeCameraPinhole() override = default;

    virtual void update(const Eigen::Ref<const MatX1> &dx);
    virtual void update_from_auxiliary(const Eigen::Ref<const MatX1> &dx);
    virtual void set_state(const Eigen::Ref<const MatX> &x) {k_state_ = x;};
    virtual void set_auxiliary_state(const Eigen::Ref<const MatX> &x) {k_auxiliary_ = x;};
    virtual const Eigen::Ref<const MatX> get_state() const {return k_state_;};
    virtual const Eigen::Ref<const MatX> get_auxiliary_state() const {return k_auxiliary_;};
    void print() const;


protected:
    Mat41 k_state_, k_auxiliary_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace CameraPinhole{
/**
 * Projection method, given a 3d point in local camera coordinates, it projects it to pixels
 * according to the camera model
 */
Mat21 project(const Mat41 &K, const Mat31 &P);
}


}

#endif /* NODECAMERAPINHOLE_HPP_ */
