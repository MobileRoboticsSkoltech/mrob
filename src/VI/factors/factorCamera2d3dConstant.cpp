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
 * factorCamera2d3dConstant.cpp
 *
 *  Created on: Aug 3, 2022
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/factors/factorCamera2d3dConstant.hpp"


using namespace mrob;


FactorCamera2d3dConstant::FactorCamera2d3dConstant(const Mat21 &observation,
            std::shared_ptr<Node> &nodePose,
            std::shared_ptr<Node> &nodePoint,
            std::shared_ptr<Node> &nodeCamera,
            const Mat2 &obsInf,
            Factor::robustFactorType robust_type):
        Factor(6,12, robust_type), obs_(observation), W_(obsInf)
{
    neighbourNodes_.emplace_back(nodePose);
    //In principle this is an anchor factor, plus we dont evaluate Jacobian
    neighbourNodes_.emplace_back(nodePoint);
    // This is a constant node (anchor) we will not calculate its Jacobian
    neighbourNodes_.emplace_back(nodeCamera);
}


void FactorCamera2d3dConstant::evaluate_residuals()
{
    Mat4 Tx = get_neighbour_nodes()->at(0)->get_state();
    Mat31 point = get_neighbour_nodes()->at(1)->get_state();
    Mat<3,4> K = get_neighbour_nodes()->at(2)->get_state();
    Mat31 p_camera = SE3(Tx).transform(point);
}
void FactorCamera2d3dConstant::evaluate_jacobians()
{

}

void FactorCamera2d3dConstant::evaluate_chi2()
{

}
void FactorCamera2d3dConstant::print() const
{

}
