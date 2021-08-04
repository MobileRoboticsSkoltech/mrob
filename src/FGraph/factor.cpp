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


matData_t Factor::evaluate_robust_weight(matData_t u, matData_t params)
{
    switch(robust_type_)
    {
        case HUBER:
            // p(u) = 1/2u^2    if u < d
            //        d(u-1/2d)
            if (u < 1.0) //XXX should this be set param?
            {
                robust_weight_ = 1.0;
            }
            else
            {
                robust_weight_ = 1.0/u;
            }
            break;
        case CAUCHY:
            robust_weight_ = 1.0/(1+u*u);
            break;
        case MCCLURE:
            params = 1+u*u;
            robust_weight_ = 1.0/params/params;
            break;
        case RANSAC:
            break;
        case QUADRATIC:
        default:
            robust_weight_ = 1.0;
            break;
    }
    return robust_weight_;
}

EigenFactor::EigenFactor(robustFactorType factor_type, uint_t potNumberNodes):
        Factor(0,0,factor_type, potNumberNodes)
{
    //Dimension zero since this is a non-parametric factor.
    //Also we don't known how many nodes will connect, so we set the second param to 0 (not-used)
}
