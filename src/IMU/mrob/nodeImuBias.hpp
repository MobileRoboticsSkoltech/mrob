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
 * nodeImuBias.hpp
 *
 *  Created on: Dec 1, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef NODEIMUBIAS_HPP_
#define NODEIMUBIAS_HPP_

#include "mrob/node.hpp"



/**
 * TODO, brief description, including state variables and update rules
 */

namespace mrob{


class NodeImuBias : public Node
{
  public:
    NodeImuBias(const Mat61 &initial_x);
    ~NodeImuBias() override;


    virtual void update(const Eigen::Ref<const MatX1> &dx) override;
    virtual void update_from_auxiliary(const Eigen::Ref<const MatX1> &dx) override;
    virtual void set_state(const Eigen::Ref<const MatX> &x) override {state_ = x;};
    virtual void set_auxiliary_state(const Eigen::Ref<const MatX> &x) override {auxiliary_state_ = x;};
    virtual const Eigen::Ref<const MatX> get_state() const override {return state_;};
    virtual const Eigen::Ref<const MatX> get_auxiliary_state() const override {return auxiliary_state_;};
    virtual void print() const override;

  protected:
    Mat61 state_; // s = [b_gyroscope[0-2], b_acceleromenter[3-5]]
    Mat61 auxiliary_state_;

};


}
#endif /* NODEIMUBIAS_HPP_ */
