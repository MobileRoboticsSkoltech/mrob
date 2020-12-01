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
 * factorBiasPropagation.hpp
 *
 *  Created on: Dec 1, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef FACTORBIASPROPAGATION_HPP_
#define FACTORBIASPROPAGATION_HPP_

#include "mrob/factor.hpp"

/**
 * TODO brief description
 */

namespace mrob{

    class FactorBiasPropagation : public Factor
    {
    public:
        FactorBiasPropagation();
        ~FactorBiasPropagation() override;

        void evaluate_residuals() override;
        void evaluate_jacobians() override;
        void evaluate_chi2() override;

        void print() const;

        const Eigen::Ref<const MatX> get_obs() const {return obs_;};
        const Eigen::Ref<const MatX1> get_residual() const {return r_;};
        const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
        const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

    protected:
        Mat61 obs_, r_;
        Mat6 W_;
        Mat6 J_;

    };

}




#endif /* FACTORBIASPROPAGATION_HPP_ */
