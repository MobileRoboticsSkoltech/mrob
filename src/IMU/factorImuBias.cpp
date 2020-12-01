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
 * factorImuBias.cpp
 *
 *  Created on: Dec 1, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factorImuBias.hpp"

using namespace mrob;


FactorImuBias::FactorImuBias(std::vector<Mat61> obs,
        std::shared_ptr<Node> &nodeOrigin, std::shared_ptr<Node> &nodeTarget,
        std::shared_ptr<Node> &nodeBias, const Mat6 &obsInf):
        Factor(6,18) //TODO, check dimensions, see factor.hpp, initiallize all variables
{

}
FactorImuBias::~FactorImuBias()
{

}

void FactorImuBias::evaluate_residuals()
{
    //TODO here we evaluate the long sequence of imu integrations
}

void FactorImuBias::evaluate_jacobians()
{
    // Follow the examples (any in Fgraph). I would create an intermediate class variable storing all accumulated rotations in evlaute
}

void FactorImuBias::evaluate_chi2()
{

}

void FactorImuBias::print() const
{

}

