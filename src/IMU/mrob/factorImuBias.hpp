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
 * factorImuBias.hpp
 *
 *  Created on: Dec 1, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTORIMUBIAS_HPP_
#define FACTORIMUBIAS_HPP_

#include "mrob/factor.hpp"
#include <vector>

/**
 * TODO brief description
 */

namespace mrob{

    class FactorImuBias : public Factor
    {
    public:
        // TODO: marsel, take a look at factor_graph_solve.cpp function get_estimated_state(), but maybe you need a vector of Mat61
        FactorImuBias(std::vector<Mat61> obs,
                std::shared_ptr<Node> &nodeOrigin, std::shared_ptr<Node> &nodeTarget,
                std::shared_ptr<Node> &nodeBias, const Mat6 &obsInf);
        ~FactorImuBias() override;

        void evaluate_residuals() override;
        void evaluate_jacobians() override;
        void evaluate_chi2() override;

        void print() const;

        const Eigen::Ref<const MatX> get_obs() const
                {assert(0 && "factorImu:get_obs: method should not be called");return Mat61::Zero();};
        const Eigen::Ref<const MatX1> get_residual() const {return r_;};
        const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
        const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

    protected:
        Mat61 r_;
        Mat3 W_;
        Mat3 J_;

    };

}


#endif /* FACTORIMUBIAS_HPP_ */
