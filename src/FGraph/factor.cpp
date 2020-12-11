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
 * factor.cpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/factor.hpp"

using namespace mrob;

Factor::Factor(uint_t dim, uint_t allNodesDim, robustFactorType factor_type, uint_t potNumberNodes):
		id_(0), dim_(dim), allNodesDim_(allNodesDim), chi2_(0), robust_type_(factor_type), robust_weight_(0.0)
{
    // Child factor must specify dimensions
    neighbourNodes_.reserve( potNumberNodes );
}

Factor::~Factor()
{
    neighbourNodes_.clear();
}


void Factor::evaluate_robust_weight(matData_t u, matData_t params)
{
    switch(robust_type_)
    {
        case HUBER:
        case CAUCHY:
        case MCCLURE:
        case RANSAC:
        case QUADRATIC:
        default:
            robust_weight_ = 1.0;
            return;
    }
}
